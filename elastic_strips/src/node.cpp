/*
Copyright (c) 2020, Manuel Beschi 
CARI Joint Research Lab
UNIBS-DIMI manuel.beschi@unibs.it
CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>

// libreria di dinamica
#include <rosdyn_core/primitives.h>

// libreria per facilitare la gestione dei sottoscrittori
#include <subscription_notifier/subscription_notifier.h>

// libreria per riordinare i vettori
#include <name_sorting/name_sorting.h>

// convertire messaggi nella libreria Eigen
#include <eigen_conversions/eigen_msg.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "elastic_strips");
  ros::NodeHandle nh;


  // leggo il modello URDF dal ros parameter server
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
  {
    ROS_ERROR("Unable to find the robot_description, do you load it?");
    return 0;
  }

  // leggo i nomi dei link di inizio e fine della catena cinematica che voglio usare
  std::string base_frame;
  std::string tool_frame;
  if (!nh.getParam("base_frame",base_frame))
  {
    ROS_ERROR("Unable to find base_frame, do you load it?");
    return 0;
  }

  if (!nh.getParam("tool_frame",tool_frame))
  {
    ROS_ERROR("Unable to find tool_frame, do you load it?");
    return 0;
  }

  // setto la gravità. Uso la libreria Eigen che consente il calcolo matriciale.
  Eigen::Vector3d gravity;
  gravity << 0,0,-9.806;


  // creo la catena cinematica
  rosdyn::ChainPtr chain=rosdyn::createChain(urdf_model,base_frame,tool_frame,gravity);

  // subscription notifier offre delle funzioni per gestire la ricezione del topic.
  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_sub(nh,"/manipulator/joint_states",10);
  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> jt_sub(nh,"/strip/joint_target",10);
  ros_helper::SubscriptionNotifier<geometry_msgs::PoseArray> poses_sub(nh,"/poses",10);
  ros_helper::SubscriptionNotifier<std_msgs::Float64> ex_ratio_sub(nh,"/execution_ratio",10);

  // se si sta usando Gazebo, il /clock è fornito da Gazebo, per dare tempo al nodo di riceverlo aspettiamo un secondo.
  ros::WallDuration(1).sleep();
  ros::spinOnce(); // ricevo i messaggi se ne è arrivato qualcuno

  // aspetto che arrivi un dato per 200 secondi
  if (!js_sub.waitForANewData(ros::Duration(200)))
  {
    ROS_ERROR("No data from /manipulator/joint_states");
    return 0;
  }

  // aspetto che arrivi un dato per 200 secondi
  if (!ex_ratio_sub.waitForANewData(ros::Duration(200)))
  {
    ROS_ERROR("No data from /execution_ratio");
    return 0;
  }

  // ottieni il messaggio
  sensor_msgs::JointState joint_states=js_sub.getData();

  // l'ordine dei giunti del JointState potrebbe essere diverso da quello della chain. riordino
  if (!name_sorting::permutationName(chain->getMoveableJointNames(), //ordine desiderato
                                    joint_states.name, //ordine attuale
                                    joint_states.position,
                                    joint_states.velocity))
  {
    ROS_ERROR("some joints are missing");
    return 0;
  }


  // inizializzo i vettori posizione, velocità e accelerazione
  Eigen::VectorXd q(chain->getActiveJointsNumber()); //posizione
  Eigen::VectorXd Dq(chain->getActiveJointsNumber()); // velocità
  //copio dati in q e Dq
  for (unsigned int idx=0;idx<chain->getActiveJointsNumber();idx++)
  {
    q(idx)=joint_states.position.at(idx); // notare la differenza nell'accedere all'elemento in Eigen::VectorXd e std::vector
    Dq(idx)=joint_states.velocity.at(idx);
  }

  // posizioni giunti nominali: quelle della traiettoria se non ci fosse l'uomo
  Eigen::VectorXd q_nom(chain->getActiveJointsNumber()); //posizione nominali
  Eigen::VectorXd Dq_nom(chain->getActiveJointsNumber()); // velocità nominali

  // per ora supponiamo q_nom costante
  q_nom.setZero();
  q_nom(2)=M_PI*0.5;
  Dq_nom.setZero();

  // target (setpoint anello di posizione del robot)
  Eigen::VectorXd q_target(chain->getActiveJointsNumber()); //posizione target
  Eigen::VectorXd Dq_target(chain->getActiveJointsNumber()); // velocità target
  Eigen::VectorXd DDq_target(chain->getActiveJointsNumber()); // accelerazione target
  Eigen::VectorXd joint_torque(chain->getActiveJointsNumber()); // coppie

  q_target=q;
  Dq_target=Dq;
  DDq_target.setZero();


  std::vector<std::string> link_names=chain->getLinksName();

  // creo una mappa che per ogni link mi da la molla corrispondente.
  // la sintassi è terribile deriva da questo:
  // https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
  std::map<std::string, Eigen::Vector3d, std::less<std::string>,
           Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d> > > link_springs;

  std::map<std::string, Eigen::Vector3d, std::less<std::string>,
           Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d> > > link_dampers;

  std::map<std::string,double> link_activation_distances;

  std::map<std::string, Eigen::Vector3d, std::less<std::string>,
           Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d> > > link_self_springs;

  std::map<std::string, Eigen::Vector3d, std::less<std::string>,
           Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d> > > link_self_dampers;

  // carico molle e smorzatori
  for (const std::string& link_name: link_names)
  {
    Eigen::Vector3d spring, damper;
    std::vector<double> spring_vec, damper_vec; // per caricare un vettore da param bisogna caricare un std::vector e poi convertirlo in Eigen::Vector
    double activation_distance;
    if ((!nh.getParam(link_name+"/spring",spring_vec)) || (!nh.getParam(link_name+"/damper",damper_vec)) || (!nh.getParam(link_name+"/activation_distance",activation_distance)))
    {
      // non è definita una molla e uno smorzatore per questo link
      spring.setZero();
      damper.setZero();
      activation_distance=0;
    }
    else
    {
      spring(0)=spring_vec.at(0);
      spring(1)=spring_vec.at(1);
      spring(2)=spring_vec.at(2);
      damper(0)=damper_vec.at(0);
      damper(1)=damper_vec.at(1);
      damper(2)=damper_vec.at(2);
    }
    ROS_INFO_STREAM("Link " << link_name <<" has:" << std::endl <<
                    "spring = " << spring.transpose() << std::endl <<
                    "damper = " << damper.transpose() << std::endl <<
                    "activation distance = "  << activation_distance);

    link_springs.insert(std::pair<std::string,Eigen::Vector3d>(link_name,spring));
    link_dampers.insert(std::pair<std::string,Eigen::Vector3d>(link_name,damper));
    link_activation_distances.insert(std::pair<std::string,double>(link_name,activation_distance));


    Eigen::Vector3d self_spring, self_damper;
    std::vector<double> self_spring_vec, self_damper_vec; // per caricare un vettore da param bisogna caricare un std::vector e poi convertirlo in Eigen::Vector

    if ((!nh.getParam(link_name+"/self_spring",self_spring_vec)) || (!nh.getParam(link_name+"/damper",self_damper_vec)))
    {
      // non è definita una molla e uno smorzatore per questo link
      self_spring.setZero();
      self_damper.setZero();
    }
    else
    {
      self_spring(0)=self_spring_vec.at(0);
      self_spring(1)=self_spring_vec.at(1);
      self_spring(2)=self_spring_vec.at(2);
      self_damper(0)=self_damper_vec.at(0);
      self_damper(1)=self_damper_vec.at(1);
      self_damper(2)=self_damper_vec.at(2);
    }
    ROS_INFO_STREAM("Link " << link_name <<" has:" << std::endl <<
                    "self_spring = " << self_spring.transpose() << std::endl <<
                    "self_damper = " << self_damper.transpose());
    link_self_springs.insert(std::pair<std::string,Eigen::Vector3d>(link_name,self_spring));
    link_self_dampers.insert(std::pair<std::string,Eigen::Vector3d>(link_name,self_damper));
  }

  Eigen::VectorXd joint_spring(chain->getActiveJointsNumber());
  Eigen::VectorXd joint_damper(chain->getActiveJointsNumber());
  std::vector<double> joint_spring_vec, joint_damper_vec;
  if ((!nh.getParam("joint_spring",joint_spring_vec)) || (!nh.getParam("joint_damper",joint_damper_vec)))
  {
    // non è definita una molla e uno smorzatore per questo link
    joint_spring.setZero();
    joint_damper.setZero();
  }
  else
  {
    for (unsigned int idx=0;idx<chain->getActiveJointsNumber();idx++)
    {
      joint_spring(idx)=joint_spring_vec.at(idx);
      joint_damper(idx)=joint_damper_vec.at(idx);
    }
  }
  ROS_INFO_STREAM("joint_spring = " << joint_spring.transpose() << std::endl <<
                  "joint_damper = " << joint_damper.transpose());



  Eigen::VectorXd max_deflection(chain->getActiveJointsNumber());
  std::vector<double> max_deflection_vec;
  if (!nh.getParam("max_joint_deflaction",joint_spring_vec))
  {
    ROS_INFO("max_joint_deflaction is not defined. set equal to 0");
    max_deflection.setConstant(0.1);
  }
  else
  {
    for (unsigned int idx=0;idx<chain->getActiveJointsNumber();idx++)
      max_deflection(idx)=max_deflection_vec.at(idx);
  }

  Eigen::VectorXd upper_limit=chain->getQMax();
  Eigen::VectorXd lower_limit=chain->getQMin();
  Eigen::VectorXd Dq_max=chain->getDQMax();
  Eigen::VectorXd DDq_max=5*Dq_max; //ipotesi

  ros::Publisher jt_pub=nh.advertise<sensor_msgs::JointState>("/manipulator/joint_target",1);
  sensor_msgs::JointState joint_target;
  joint_target=joint_states;

  double st=2e-3;
  ros::Rate rate(1.0/st); // cicla a 2 ms


  // vettore dei punti rappresentanti l'uomo
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> human_points_in_b;

  while (ros::ok())
  {

    ros::spinOnce(); // ricevo i messaggi se ne è arrivato qualcuno

    double execution_ratio=ex_ratio_sub.getData().data; // std_msgs/Float64 ha un campo chiamato data che contiene il double

    joint_states=js_sub.getData();
    if (!name_sorting::permutationName(chain->getMoveableJointNames(),
                                      joint_states.name,
                                      joint_states.position,
                                      joint_states.velocity))
    {
      ROS_ERROR("some joints are missing");
      return 0;
    }
    //copio dati in q e Dq
    for (unsigned int idx=0;idx<chain->getActiveJointsNumber();idx++)
    {
      q(idx)=joint_states.position.at(idx); // notare la differenza nell'accedere all'elemento in Eigen::VectorXd e std::vector
      Dq(idx)=joint_states.velocity.at(idx);
    }
    if (jt_sub.isANewDataAvailable())
    {
      sensor_msgs::JointState joint_setpoint=jt_sub.getData();
      if (!name_sorting::permutationName(chain->getMoveableJointNames(),
                                        joint_setpoint.name,
                                        joint_setpoint.position,
                                        joint_setpoint.velocity))
      {
        ROS_ERROR("some joints are missing");
        return 0;
      }
      for (unsigned int idx=0;idx<chain->getActiveJointsNumber();idx++)
      {
        q_nom(idx)=joint_setpoint.position.at(idx); // notare la differenza nell'accedere all'elemento in Eigen::VectorXd e std::vector
        Dq_nom(idx)=joint_setpoint.velocity.at(idx);
      }
    }



    if (poses_sub.isANewDataAvailable())
    {
      human_points_in_b.clear();
      geometry_msgs::PoseArray human_poses_in_b=poses_sub.getData();  // punti dell'umano nel frame base (b)
      if (base_frame.compare(human_poses_in_b.header.frame_id)!=0)
      {
        ROS_ERROR("poses are not in %s frame, but in %s",base_frame.c_str(),human_poses_in_b.header.frame_id.c_str());
        return 0;
      }
      // per tutti le pose contenute in human poses
      for (const geometry_msgs::Pose pose: human_poses_in_b.poses)
      {
        Eigen::Vector3d human_point_in_b; // punto dell'umano nel frame base (b)
        tf::pointMsgToEigen(pose.position,human_point_in_b);
        human_points_in_b.push_back(human_point_in_b);
      }
    }

    joint_torque.setZero();

    for (const std::string& link_name: link_names)
    {
      Eigen::Vector3d spring=link_springs.at(link_name);
      Eigen::Vector3d damper=link_dampers.at(link_name);
      double activation_distance=link_activation_distances.at(link_name);

      Eigen::Vector3d self_spring=link_self_springs.at(link_name);
      Eigen::Vector3d self_damper=link_self_dampers.at(link_name);

      // la matrice Affine3d rappresenta una matrica 4x4 di transformazione.
      // ATTENZIONE: la matrice di rotazione si chiama linear() perché è un'operazione lineare nello spazio affine....
      // la translazione è translation()
      Eigen::Affine3d T_b_l=chain->getTransformationLink(q_target,link_name);
      Eigen::Vector3d o_l_in_b=T_b_l.translation(); // origine di l vista in b
      Eigen::Vector6d twist_on_l_in_b=chain->getTwistLink(q_target,Dq_target,link_name); // twist (vel lin, vel rot) del link l visto nel frame b

      // posa nominale del link nel frame base
      Eigen::Affine3d T_b_lnom=chain->getTransformationLink(q_nom,link_name);
      Eigen::Vector3d o_lnom_in_b=T_b_l.translation(); // origine di lnom vista in b
      Eigen::Vector6d twist_on_lnom_in_b=chain->getTwistLink(q_nom,Dq_nom,link_name); // twist (vel lin, vel rot) del link lnom visto nel frame b

      // cwise è l'equivalente di .* in matlab
      Eigen::Vector3d self_elastic_force_on_l_in_b=self_spring.cwiseProduct(o_lnom_in_b-o_l_in_b);

      // implementa forze damping
      Eigen::Vector3d self_damping_force_on_l_in_b;
      self_damping_force_on_l_in_b=self_damper.cwiseProduct(twist_on_lnom_in_b.head(3)-twist_on_l_in_b.head(3));



      Eigen::Vector3d elastic_force_on_l_in_b;
      elastic_force_on_l_in_b.setZero();
      Eigen::Vector3d damping_force_on_l_in_b;
      damping_force_on_l_in_b.setZero();

      if (activation_distance>0)
      {
        double min_distance=std::numeric_limits<double>::infinity();
        for (const Eigen::Vector3d& human_point_in_b: human_points_in_b)
        {
          // implementa forze repulsive e sommale
          double distance=(human_point_in_b-o_l_in_b).norm();
          if (distance<min_distance)
            min_distance=distance;
          if (distance<activation_distance)
          {
            Eigen::Vector3d versor=(human_point_in_b-o_l_in_b).normalized();
            // esempio=controllare segni e modificare nel caso
            elastic_force_on_l_in_b+=spring.cwiseProduct(versor*(distance-activation_distance));
          }
        }
        ROS_DEBUG_THROTTLE(0.1,"minimum distance between human and %s: %f",link_name.c_str(),min_distance);
      }

      Eigen::Vector3d force_on_l_in_b=self_elastic_force_on_l_in_b+self_damping_force_on_l_in_b+
                                      elastic_force_on_l_in_b+damping_force_on_l_in_b;

      Eigen::Vector3d torque_on_l_in_b;
      torque_on_l_in_b.setZero();

      Eigen::Vector6d wrench_on_l_in_b;
      wrench_on_l_in_b << force_on_l_in_b, torque_on_l_in_b;

      // Jacobiano del link l visto nel frame b. Le prime tre righe sono la parte legata alle traslazioni, le ultime 3 righe quelle legate alle rotazioni
      Eigen::Matrix6Xd jacobian_of_l_in_b=chain->getJacobianLink(q_target,link_name);

      Eigen::VectorXd joint_torque_due_to_l=jacobian_of_l_in_b.transpose()*wrench_on_l_in_b;

      joint_torque+=joint_torque_due_to_l;
    }

    joint_torque+=joint_spring.cwiseProduct(q_nom-q_target);
    joint_torque+=joint_damper.cwiseProduct(Dq_nom-Dq_target);



    // coppie = B(q)*DDq+nl(q,Dq)
    // dove nl contiene le coppie di Coriolis e dovute alla gravità
    Eigen::MatrixXd joint_inertia=chain->getJointInertia(q_target);
    Eigen::VectorXd nl=chain->getJointTorqueNonLinearPart(q_target,Dq_target);

    // se interessano le coppie dovute alla gravita
    Eigen::VectorXd grav_torque=chain->getJointTorqueNonLinearPart(q_target,0*Dq_target);
    // quelle di Coriolis si trovano per differenza

    // compenso gravità
    joint_torque+=grav_torque;

    // dinamica diretta
    DDq_target=joint_inertia.inverse()*(joint_torque-nl);

    for (unsigned int idx=0;idx<chain->getActiveJointsNumber();idx++)
    {



      double deformation_ratio=1-2*std::abs(0.5-execution_ratio); //deforma fino al limite se execution ratio=0.5, non deforma se execution ratio=0 oppure =1
      double q_max=std::min(q_nom(idx)+deformation_ratio*max_deflection(idx),upper_limit(idx));
      double q_min=std::max(q_nom(idx)-deformation_ratio*max_deflection(idx),lower_limit(idx));


      // computing breacking distance
      double t_break=std::abs(Dq_target(idx))/DDq_max(idx);
      double breaking_distance=0.5*DDq_max(idx)*std::pow(t_break,2.0);

      if (Dq_target(idx)>Dq_max(idx))
        Dq_target(idx)=Dq_max(idx);
      else if (Dq_target(idx)<-Dq_max(idx))
          Dq_target(idx)=-Dq_max(idx);

      if (std::abs(q_max-q_min)<0.5*DDq_max(idx)*std::pow(st,2.0)) // se l'upper e il lower bound sono molto vicini sequi traiettoria nominale
      {
        q_target(idx)=q_nom(idx);
        Dq_target(idx)=Dq_nom(idx);
        DDq_target(idx)=0;
      }
      else if (q_target(idx) >= (q_max-breaking_distance)) // frena per non superare upper bound
      {
        if (Dq_target(idx)>0)
        {
          ROS_WARN_THROTTLE(2,"Breaking, maximum limit approaching on joint %u",idx);
          DDq_target(idx)=-DDq_max(idx);
        }
      }
      else if (q_target(idx) <= (q_min + breaking_distance)) // frena per non superare loew bound
      {
        if (Dq_target(idx) < 0)
        {
          ROS_WARN_THROTTLE(2,"Breaking, minimum limit approaching on joint %u",idx);
          DDq_target(idx)=+DDq_max(idx);
        }
      }

      q_target(idx)+=Dq_target(idx)*st+0.5*DDq_target(idx)*std::pow(st,2);
      Dq_target(idx)+=DDq_target(idx)*st;

      joint_target.position.at(idx)=q_target(idx);
      joint_target.velocity.at(idx)=Dq_target(idx);
    }
    joint_target.header.stamp=ros::Time::now(); // aggiorno tempo messaggio (per log, print)

    jt_pub.publish(joint_target);

    rate.sleep();
  }


  return 0;
}
