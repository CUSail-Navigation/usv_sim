#include "../include/foil_dynamics_plugin/foil_dynamics_plugin.h"

#include <algorithm>
#include <string>

#include <gazebo/common/Assert.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/transport/transport.hh>
#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN (Foil_Dynamics_Plugin)

/////////////////////////////////////////////////
Foil_Dynamics_Plugin::Foil_Dynamics_Plugin () : rho (1000.1)
{
	ROS_INFO ("------------------------------Foil_Dynamics_Plugin OBJECT CREATED!!!!");
	this->cp = ignition::math::Vector3d (0, 0, 0);
	this->forward = ignition::math::Vector3d (1, 0, 0);
	this->upward = ignition::math::Vector3d (0, 0, 1);
	this->area = 1.0;
	this->mult_lift = 1.5;
	this->mult_drag = 1.0;
	this->oldTime = ros::Time::now();
}

/////////////////////////////////////////////////
void
Foil_Dynamics_Plugin::Load (physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	ROS_INFO ("------------------------------Foil_Dynamics_Plugin loaded!!!!");

	GZ_ASSERT (_model, "Foil_Dynamics_Plugin _model pointer is NULL");
	GZ_ASSERT (_sdf, "Foil_Dynamics_Plugin _sdf pointer is NULL");
	this->model = _model;
	this->modelName = _model->GetName ();
	this->sdf = _sdf;
	rosnode_ = ros::NodeHandle (modelName);

	this->world = this->model->GetWorld ();
	GZ_ASSERT (this->world, "Foil_Dynamics_Plugin world pointer is NULL");

	this->physics = this->world->Physics  ();
	GZ_ASSERT (this->physics, "Foil_Dynamics_Plugin physics pointer is NULL");

	GZ_ASSERT (_sdf, "Foil_Dynamics_Plugin _sdf pointer is NULL");


	if (_sdf->HasElement ("cp"))
		this->cp = _sdf->Get < ignition::math::Vector3d > ("cp");

	// blade forward (-drag) direction in link frame
	if (_sdf->HasElement ("forward"))
		this->forward = _sdf->Get < ignition::math::Vector3d > ("forward");

	// blade upward (+lift) direction in link frame
	if (_sdf->HasElement ("upward"))
		this->upward = _sdf->Get < ignition::math::Vector3d > ("upward");

	if (_sdf->HasElement ("area"))
		this->area = _sdf->Get<double> ("area");

	if (_sdf->HasElement ("mult_lift"))
			this->mult_lift = _sdf->Get<double> ("mult_lift");

	if (_sdf->HasElement ("mult_drag"))
				this->mult_lift = _sdf->Get<double> ("mult_drag");

	if (_sdf->HasElement ("fluid_density"))
		this->rho = _sdf->Get<double> ("fluid_density");

	if (_sdf->HasElement ("link_name"))
	{
		sdf::ElementPtr elem = _sdf->GetElement ("link_name");
		this->linkName = elem->Get<std::string> ();
		this->link = this->model->GetLink (this->linkName);
	}

	if (_sdf->HasElement ("link_type"))
	{
		sdf::ElementPtr elem = _sdf->GetElement ("link_type");
		this->linkType = elem->Get<std::string> ();
	}
	if (_sdf->HasElement ("joint_name"))
	{
		sdf::ElementPtr elem = _sdf->GetElement ("joint_name");
		this->jointName = elem->Get<std::string> ();
		this->joint = this->model->GetJoint (this->jointName);
		std::cerr << "Joint name: [" << this->jointName << "\n";
		std::cerr << "Joint: [" << this->joint->GetName () << "\n";
	}
	if (_sdf->HasElement ("fluidVelocity"))
	{
		sdf::ElementPtr elem = _sdf->GetElement ("fluidVelocity");
		this->fluidVelocity = elem->Get<std::string> ();
		std::cerr << "fluidVelocity: [" << this->fluidVelocity << "\n";
	}

	waterCurrent = ignition::math::Vector3d (0, 0, 0);
	float wind_value_x;
	float wind_value_y;
	float wind_value_z;
	running = false;
	if (fluidVelocity.compare ("global") == 0)
	{

		if (rosnode_.getParam ("/uwsim/wind/x", wind_value_x) & rosnode_.getParam ("/uwsim/wind/y", wind_value_y))
		{
			this->wind = ignition::math::Vector3d (wind_value_x, wind_value_y, 0);
			ROS_WARN("\n WIND TYPE IS GLOBAL wind %f, %f, %f", this->wind.X(), this->wind.Y(), this->wind.Z());
		}
		else
		{
			ROS_INFO ("Foil_Dynamics_Plugin:: Sail plugin error: Cant find value of /uwsim/wind in param server");
		}
	}
	else if (fluidVelocity.compare ("local") == 0)
	{
		if (this->linkType.compare ("sail") == 0)
		{
			std::cerr << "\n initializing wind service client";
			velocity_serviceClient_ = rosnode_.serviceClient < wind_current::GetSpeed > ("/windCurrent");
			std::cerr << " ... done";
			running = true;
			the_thread = std::thread (&Foil_Dynamics_Plugin::WindThreadLoop, this);
		}
		else
		{
			std::cerr << "\n initializing WATER service client";
			velocity_serviceClient_ = rosnode_.serviceClient < water_current::GetSpeed > ("/waterCurrent");
			std::cerr << " ... done";
			running = true;
			the_thread = std::thread (&Foil_Dynamics_Plugin::WaterThreadLoop, this);
		}
	}
}

/////////////////////////////////////////////////
void
Foil_Dynamics_Plugin::Init ()
{
	std::cerr << "\n ----------- Foil_Dynamics_Plugin::init: type: " << this->linkType << " linkName: " << this->linkName;
	current_subscriber_ = rosnode_.subscribe ("/gazebo/current", 1, &Foil_Dynamics_Plugin::ReadWaterCurrent, this);
	wind_speed_subscriber_ = rosnode_.subscribe ("/sailboat/uwsim/wind_speed", 1, &Foil_Dynamics_Plugin::ReadWindSpeed, this);
	this->updateConnection = event::Events::ConnectWorldUpdateBegin (boost::bind (&Foil_Dynamics_Plugin::OnUpdate, this));

	std::cerr << "\n compare to sail: " << this->linkType.compare ("sail");
	if (this->linkType.compare ("sail") == 0)
	{
	//	std::string topic = "/" + this->model->GetName () + "/angleLimits";
	//	this->angleLimits_subscriber = rosnode_.subscribe (topic, 1, &Foil_Dynamics_Plugin::ropeSimulator, this);
	}
}

void
Foil_Dynamics_Plugin::OnUpdate ()
{
	if (oldTime > ros::Time::now())
	{
		oldTime = ros::Time::now();
		return;
	}
	if (this->linkType.compare ("rudder") == 0)
		this->OnUpdateRudder ();
	else if (this->linkType.compare ("keel") == 0)
		this->OnUpdateKeel ();
	else if (this->linkType.compare ("sail") == 0)
		this->OnUpdateSail ();
	oldTime = ros::Time::now();
}

/////////////////////////////////////////////////
void
Foil_Dynamics_Plugin::OnUpdateRudder ()
{
	// get linear velocity at cp in inertial frame
	ignition::math::Vector3d vel = this->link->WorldLinearVel (this->cp) - waterCurrent;

	if (vel.Length () <= 0.01)
		return;
	if (vel.Length () > 5)
	{
		//std::cerr<<"\n OnUpdateRudder::vel: "<<vel<<" lengtH: "<<vel.Length()<<" oldTime: "<<oldTime<<" > "<<ros::Time::now();
		//std::cerr<<"\n cp "<<this->cp<<" pose: "<<this->link->GetWorldPose ();
		return;
	}
	// pose of body
	ignition::math::Pose3d pose = this->link->WorldPose ();

	// rotate forward and upward vectors into inertial frame
	ignition::math::Vector3d forwardI = pose.Rot().RotateVector (this->forward);
	ignition::math::Vector3d upwardI = pose.Rot().RotateVector (this->upward);
	//std::cerr<<"pose: "<<pose<<" forwardI: "<<forwardI<<" upwardI: "<<upwardI<<"\n";

	// ldNormal vector to lift-drag-plane described in inertial frame
	ignition::math::Vector3d ldNormal = forwardI.Cross (upwardI).Normalize ();


	// angle of attack
	ignition::math::Vector3d velInLDPlane = ldNormal.Cross (vel.Cross (ldNormal));

	// get direction of drag
	ignition::math::Vector3d dragDirection = -velInLDPlane;
	dragDirection.Normalize ();

	// get direction of lift
	ignition::math::Vector3d liftDirection = ldNormal.Cross (velInLDPlane);
	liftDirection.Normalize ();
	//std::cerr<<"vel: "<<vel<<"("<<vel.GetLength()<<") liftDirection: "<<liftDirection<<" dragDirection: "<<dragDirection<<"\n";

	// get direction of moment
	ignition::math::Vector3d momentDirection = ldNormal;

	double cosAlpha = ignition::math::clamp (forwardI.Dot (velInLDPlane) / (forwardI.Length () * velInLDPlane.Length ()),
	                               -1.0, 1.0);

	// get sign of alpha
	// take upwards component of velocity in lift-drag plane.
	// if sign == upward, then alpha is negative
	double alphaSign = -upwardI.Dot (velInLDPlane) / (upwardI.Length () + velInLDPlane.Length ());

	// double sinAlpha = sqrt(1.0 - cosAlpha * cosAlpha);
	if (alphaSign > 0.0)
		this->alpha =  acos (cosAlpha);
	else
		this->alpha = - acos (cosAlpha);

	// compute dynamic pressure
	double speedInLDPlane = velInLDPlane.Length ();
	double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

	// compute cl at cp, check for stall, correct for sweep
	double cl;
	cl = this->mult_lift * sin (2*this->alpha);
	// compute lift force at cp
	ignition::math::Vector3d lift = cl * q * this->area * liftDirection;

	// compute cd at cp, check for stall, correct for sweep
	double cd;

	cd =  this->mult_drag * (1 - cos (2 * this->alpha));
	// make sure drag is positive
	cd = fabs (cd);

	// drag at cp
	ignition::math::Vector3d drag = cd * q * this->area * dragDirection;

	// force and torque about cg in inertial frame
	ignition::math::Vector3d force = lift + drag;
 //ROS_ERROR("liftDirection: %f,%f,%f lift: %f, %f drag: (%f, %f)",liftDirection.x,liftDirection.y,liftDirection.z,lift.x,lift.y,drag.x,drag.y);
//	ROS_ERROR("a: %f inputCL: %f inputCD: %f v: %f", this->alpha, sin (2*this->alpha), (1 - cos (2 * this->alpha)), speedInLDPlane);
//	std::cerr<<"\n Lift: "<<lift<<" Drag: "<<drag;
//	std::cerr<<"\n rho: "<<this->rho<<" Area: "<<this->area;
//	std::cerr<<" totalForce: "<<(lift+drag)<<"\n";

	ignition::math::Vector3d torque = (lift + drag)*(this->cp - this->link->GetInertial ()->CoG ());

	// apply forces at cg (with torques for position shift)
	//std::cerr<<"\n forceRudder["<<force.GetLength()<<"]: "<<force;
	this->link->AddForceAtRelativePosition (force, this->cp);
	//this->link->AddTorque(torque);
}




void
Foil_Dynamics_Plugin::OnUpdateKeel ()
{

	ignition::math::Vector3d vel = this->link->WorldLinearVel (this->cp) - waterCurrent;

	if (vel.Length () <= 0.01)
		return;

	// pose of body
	ignition::math::Pose3d pose = this->link->WorldPose ();

	// rotate forward and upward vectors into inertial frame
	ignition::math::Vector3d forwardI = pose.Rot().RotateVector (this->forward);
	ignition::math::Vector3d upwardI = pose.Rot().RotateVector (this->upward);

	// ldNormal vector to lift-drag-plane described in inertial frame
	ignition::math::Vector3d ldNormal = forwardI.Cross (upwardI).Normalize ();

	// check sweep (angle between vel and lift-drag-plane)
	double sinSweepAngle = ldNormal.Dot (vel) / vel.Length ();

	// get cos from trig identity
	double cosSweepAngle2 = (1.0 - sinSweepAngle * sinSweepAngle);
	this->sweep = asin (sinSweepAngle);

	// truncate sweep to within +/-90 deg
	while (fabs (this->sweep) > 0.5 * M_PI)
		this->sweep = this->sweep > 0 ? this->sweep - M_PI : this->sweep + M_PI;

	// angle of attack is the angle between
	// vel projected into lift-drag plane
	//  and
	// forward vector
	//
	// projected = ldNormal Xcross ( vector Xcross ldNormal)
	//
	// so,
	// velocity in lift-drag plane (expressed in inertial frame) is:
	ignition::math::Vector3d velInLDPlane = ldNormal.Cross (vel.Cross (ldNormal));

	// get direction of drag
	ignition::math::Vector3d dragDirection = -velInLDPlane;
	dragDirection.Normalize ();

	// get direction of lift
	ignition::math::Vector3d liftDirection = ldNormal.Cross (velInLDPlane);
	liftDirection.Normalize ();
	//std::cerr<<"\n liftDirection: "<<liftDirection;

	// get direction of moment
	ignition::math::Vector3d momentDirection = ldNormal;

	double cosAlpha = ignition::math::clamp (forwardI.Dot (velInLDPlane) / (forwardI.Length () * velInLDPlane.Length ()),
	                               -1.0, 1.0);

	// get sign of alpha
	// take upwards component of velocity in lift-drag plane.
	// if sign == upward, then alpha is negative
	double alphaSign = -upwardI.Dot (velInLDPlane) / (upwardI.Length () + velInLDPlane.Length ());

	// double sinAlpha = sqrt(1.0 - cosAlpha * cosAlpha);
	if (alphaSign > 0.0)
		this->alpha = + acos (cosAlpha);
	else
		this->alpha = - acos (cosAlpha);

	// normalize to within +/-90 deg
	while (fabs (this->alpha) > M_PI)
		this->alpha = this->alpha > 0 ? this->alpha - 2 * M_PI : this->alpha + 2 * M_PI;

	// compute dynamic pressure
	double speedInLDPlane = velInLDPlane.Length ();
	double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

	// compute cl at cp, check for stall, correct for sweep
	double cl;
	cl = 8 * sin (2 * this->alpha);
	// compute lift force at cp
	ignition::math::Vector3d lift = cl * q * this->area * liftDirection;

	// compute cd at cp, check for stall, correct for sweep
	double cd;
	// make sure drag is positive
	//cd = fabs(cd);

	cd = 4 * (1 - cos (2 * this->alpha));
	// drag at cp
	ignition::math::Vector3d drag = cd * q * this->area * dragDirection;

	// compute cm at cp, check for stall, correct for sweep
	double cm;

	// reset cm to zero, as cm needs testing
	cm = 0.0;

	// compute moment (torque) at cp
	ignition::math::Vector3d moment = cm * q * this->area * momentDirection;

	// moment arm from cg to cp in inertial plane
	ignition::math::Vector3d momentArm = pose.Rot().RotateVector (this->cp - this->link->GetInertial ()->CoG ());

	// force and torque about cg in inertial frame
	ignition::math::Vector3d force = lift + drag;
	// + moment.Cross(momentArm);

	ignition::math::Vector3d torque = moment;

	//std::cerr<<"\n forceKeel["<<force.GetLength()<<"]: "<<force;
	// apply forces at cg (with torques for position shift)
	this->link->AddForceAtRelativePosition (force, this->cp);

}

void
Foil_Dynamics_Plugin::OnUpdateSail ()
{

	//this->joint->SetLowStop (0, gazebo::math::Angle (-this->angle));
	//this->joint->SetHighStop (0, gazebo::math::Angle (this->angle));
	ignition::math::Vector3d aw = this->wind - this->link->WorldLinearVel (this->cp);
	if (aw.Length () <= 0.01)
		return;

	// pose of body
	ignition::math::Pose3d pose = this->link->WorldPose ();

	// rotate forward and upward vectors into inertial frame
	ignition::math::Vector3d forwardI = pose.Rot().RotateVector (this->forward); //xb
	ignition::math::Vector3d upwardI = pose.Rot().RotateVector (this->upward);   //yb

	// ldNormal vector to lift-drag-plane described in inertial frame
	ignition::math::Vector3d ldNormal = forwardI.Cross (upwardI).Normalize ();
	// TODOS ESSES PRODUTOS VETORIAIS SÃO PRA PEGAR OS VETORES PERPENDICULARES

	//ignition::math::Vector3d velInLDPlane = ldNormal.Cross(aw.Cross(ldNormal)); // isso é igual ao vel, só que escalado????
	ignition::math::Vector3d velInLDPlane = aw;
	// get direction of drag
	ignition::math::Vector3d dragDirection = velInLDPlane;
	dragDirection.Normalize ();

	// get direction of lift
	// ignition::math::Vector3d liftDirection = ldNormal.Cross(velInLDPlane);
	ignition::math::Vector3d liftDirection = -ldNormal.Cross (velInLDPlane);
	liftDirection.Normalize ();

	// get direction of moment
	ignition::math::Vector3d momentDirection = ldNormal;

	double cosAlpha = ignition::math::clamp (forwardI.Dot (velInLDPlane) / (forwardI.Length () * velInLDPlane.Length ()),
	                               -1.0, 1.0);

	// get sign of alpha
	// take upwards component of velocity in lift-drag plane.
	// if sign == upward, then alpha is negative
	double alphaSign = -upwardI.Dot (velInLDPlane) / (upwardI.Length () + velInLDPlane.Length ());

	// double sinAlpha = sqrt(1.0 - cosAlpha * cosAlpha);
	if (alphaSign > 0.0)
		this->alpha = acos (cosAlpha);
	else
		this->alpha = -acos (cosAlpha);

	// compute dynamic pressure
	double speedInLDPlane = velInLDPlane.Length ();
	double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

	// compute cl at cp, check for stall, correct for sweep

	double cl;

	//cl = 8 * sin (2 * this->alpha);
	cl = 8 * sin (2 * this->alpha);
	// compute lift force at cp
	ignition::math::Vector3d lift = cl * q * this->area * liftDirection;

	// compute cd at cp, check for stall, correct for sweep
	double cd;
	// make sure drag is positive
	//cd = fabs(cd);

	cd = 4 * (1 - cos (2 * this->alpha));
	// drag at cp
	ignition::math::Vector3d drag = cd * q * this->area * dragDirection;

	// compute cm at cp, check for stall, correct for sweep
	double cm;
	// reset cm to zero, as cm needs testing
	cm = 0.0;

	// compute moment (torque) at cp
	ignition::math::Vector3d moment = cm * q * this->area * momentDirection;

	// moment arm from cg to cp in inertial plane
	ignition::math::Vector3d momentArm = pose.Rot().RotateVector (this->cp - this->link->GetInertial ()->CoG ());

	// force and torque about cg in inertial frame
	ignition::math::Vector3d force = lift + drag;
	// + moment.Cross(momentArm);

	ignition::math::Vector3d torque = moment;

	//std::cerr<<"\n forceSail["<<force.GetLength()<<"]: "<<force;
	// apply forces at cg (with torques for position shift)
	this->link->AddForceAtRelativePosition (force, this->cp);
//std::cerr<<"\n force: "<<force;
//std::cerr<<"\n aw:"<<aw.GetLength ()<<", position: "<<this->cp;
//std::cerr<<"\n aw:"<<aw.GetLength ()<<" force: "<<force.GetLength();
//std::cerr<<"\n aw:"<<aw.GetLength ()<<" lift: "<<lift<<" drag: "<<drag;
//std::cerr<<"\n aw:"<<aw.GetLength ()<<" lift: "<<lift.GetLength ()<<" drag: "<<drag.GetLength ()<<" cl: "<<cl<<" area: "<<this->area<<" q: "<<q<<" rho: "<<this->rho<<" speedInLDPlane: "<<speedInLDPlane;
//std::cerr<<"\n aw: "<<aw;
}

void
Foil_Dynamics_Plugin::ReadWaterCurrent (const geometry_msgs::Vector3::ConstPtr& _msg)
{
	waterCurrent.X(_msg->x);
	waterCurrent.Y(_msg->y);
	waterCurrent.Z(_msg->z);
}

void
Foil_Dynamics_Plugin::WaterThreadLoop ()
{
	ros::Rate r (10);
	while (running)
	{
		ignition::math::Pose3d pose = this->link->WorldCoGPose ();
		water_current::GetSpeed srv;
		srv.request.x = pose.Pos().X();
		srv.request.y = pose.Pos().Y();
		if (velocity_serviceClient_.call (srv))
		{
			waterCurrent.X(srv.response.x);
			waterCurrent.Y( srv.response.y);
			
		}
		else
		{
			ROS_WARN ("Foil_Dynamics_Plugin::Failed to call service waterCurrent %s", this->modelName.c_str ());
			ROS_WARN (" fluidVelocity: %s",fluidVelocity.c_str());
			ros::Rate s (1);
			s.sleep ();
		}
		r.sleep ();
	}
}

void
Foil_Dynamics_Plugin::ReadWindSpeed (const std_msgs::Float64MultiArray& _msg)
{
	if(sizeof(_msg.data)) {
		wind.X(_msg.data[0]);
		wind.Y(_msg.data[1]);
		ROS_INFO("WIND SPEED IS NOW %f %f %f", wind.X(), wind.Y(), wind.Z());
	}
}

void
Foil_Dynamics_Plugin::WindThreadLoop ()
{
	ros::Rate r (10);
	while (running)
	{
		ignition::math::Pose3d pose = this->link->WorldCoGPose ();
		wind_current::GetSpeed srv;
		srv.request.x = pose.Pos().X();
		srv.request.y = pose.Pos().Y();
		if (velocity_serviceClient_.call (srv))
		{
			wind.X( srv.response.x);
			wind.Y( srv.response.y);
			
		}
		else
		{
			ROS_WARN ("Failed to call service windCurrent %s", this->modelName.c_str ());

			ros::Rate s (1);
			s.sleep ();
		}
		r.sleep ();
	}
}

