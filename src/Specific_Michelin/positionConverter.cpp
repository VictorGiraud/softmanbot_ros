#include "positionConverter.hpp"

pose robot1BasePosition(0, -0.8, 1.12);//TODO : mesurer, vérifier, tester. C'est pour me donner une idée 
pose robot2BasePosition(0, 0.8, 1.12);//TODO : mesurer, vérifier, tester. C'est pour me donner une idée

pose cameraBasePosition(0, 0.4, 0.5);


pose EndEffector1ToUnifiedPosition(pose EndEffectorPose)
{
	Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(M_PI, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());//Coller a la conf du robot

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();

	//Build transformation matrix. Don't know what order i want, cf Kevin
	Eigen::Vector3d T(robot1BasePosition.x, robot1BasePosition.y, robot1BasePosition.z);
	Eigen::Matrix4d Trans; // Your Transformation Matrix
	Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	Trans.block<3,3>(0,0) = rotationMatrix;
	Trans.block<3,1>(0,3) = T;

	//All the above stuff should be computed at compile time. Sylvio dit constexpr.

	//Get the absolute pose using transformation matrix
	Eigen::Vector4d inputVector(EndEffectorPose.x, EndEffectorPose.y, EndEffectorPose.z, 1);
	Eigen::Vector4d resultHomogeneousVector = Trans*inputVector;

	//Get the absolute orientation using rotation matrix
	Eigen::Quaternion<double> qEndEffector = RPYToQuaternion(EndEffectorPose);
		

	pose retval = quaternionToRPY(qEndEffector * q);
	retval.x = resultHomogeneousVector(0);
	retval.y = resultHomogeneousVector(1); 
	retval.z = resultHomogeneousVector(2); 
	return retval;
}

pose EndEffector2ToUnifiedPosition(pose EndEffectorPose)
{
	/*EndEffectorPose.y = -EndEffectorPose.y;
	EndEffectorPose.z = -EndEffectorPose.z;
	EndEffectorPose += robot2BasePosition;
	return EndEffectorPose;*/

	//Get the rotation from the frame. Put it in a matrix.
	Eigen::AngleAxisd rollAngle(M_PI, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(M_PI, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());//Coller a la conf du robot

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();

	//Build transformation matrix. Don't know what order i want, cf Kevin
	Eigen::Vector3d T(robot2BasePosition.x, robot2BasePosition.y, robot2BasePosition.z);
	Eigen::Matrix4d Trans; // Your Transformation Matrix
	Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	Trans.block<3,3>(0,0) = rotationMatrix;
	Trans.block<3,1>(0,3) = T;

	//Get the absolute pose using transformation matrix
	Eigen::Vector4d inputVector(EndEffectorPose.x, EndEffectorPose.y, EndEffectorPose.z, 1);
	Eigen::Vector4d resultHomogeneousVector = Trans*inputVector;

	//Get the absolute orientation using rotation matrix
	Eigen::Quaternion<double> qEndEffector = RPYToQuaternion(EndEffectorPose);
		

	pose retval = quaternionToRPY(qEndEffector * q);
	retval.x = resultHomogeneousVector(0);
	retval.y = resultHomogeneousVector(1); 
	retval.z = resultHomogeneousVector(2); 
	return retval;
}

pose CameraToUnifiedPosition(pose CameraPose)
{
	//Data from camera calibration code of reza
	Eigen::Matrix4d Trans;
	Trans(0,0) = -0.098671;
	Trans(0,1) = -0.994916;
	Trans(0,2) = -0.020143;
	Trans(0,3) = 0.034521;
	Trans(1,0) = -0.989108;
	Trans(1,1) = 0.100277;
	Trans(1,2) = -0.107745;
	Trans(1,3) = -0.125868;
	Trans(2,0) = 0.109217;
	Trans(2,1) = 0.009293;
	Trans(2,2) = -0.993974;
	Trans(2,3) = 0.490025;
	Trans(3,0) = 0;
	Trans(3,1) = 0;
	Trans(3,2) = 0;
	Trans(3,3) = 1;

	//TODO : Offset marker
	
	Eigen::Vector4d inputVector(CameraPose.x, CameraPose.y, CameraPose.z, 1);
	Eigen::Vector4d resultHomogeneousVector = Trans.inverse()*inputVector;
	pose retval;
	retval.x = resultHomogeneousVector(0) - 0.2;//0.2 is offset of marker
	retval.y = resultHomogeneousVector(1); 
	retval.z = resultHomogeneousVector(2); 
	return retval;	
}

pose UnifiedPositionToEndEffector1(pose unifiedPose)
{
	//Just have to inverse the prvious stuff

	Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(M_PI, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());//Coller a la conf du robot

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();

	//Build transformation matrix. Don't know what order i want, cf Kevin
	Eigen::Vector3d T(robot1BasePosition.x, robot1BasePosition.y, robot1BasePosition.z);
	Eigen::Matrix4d Trans; // Your Transformation Matrix
	Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	Trans.block<3,3>(0,0) = rotationMatrix;
	Trans.block<3,1>(0,3) = T;

	//All the above stuff should be computed at compile time. Sylvio dit constexpr.

	//Get the absolute pose using transformation matrix
	Eigen::Vector4d inputVector(unifiedPose.x, unifiedPose.y, unifiedPose.z, 1);
	Eigen::Vector4d resultHomogeneousVector = Trans.inverse()*inputVector;

	//Get the absolute orientation using rotation matrix
	Eigen::Quaternion<double> qUnifiedPose = RPYToQuaternion(unifiedPose);
		

	pose retval = quaternionToRPY(qUnifiedPose * q.conjugate());
	retval.x = resultHomogeneousVector(0);
	retval.y = resultHomogeneousVector(1); 
	retval.z = resultHomogeneousVector(2); 
	return retval;


}

pose UnifiedPositionToEndEffector2(pose unifiedPose)
{
	/*pose unifiedPose = EndEffectorPose;
	unifiedPose -= robot2BasePosition;
	unifiedPose.y = - unifiedPose.y;
	unifiedPose.z = - unifiedPose.z;
	return unifiedPose;*/

	
	//Get the rotation from the frame. Put it in a matrix.
	Eigen::AngleAxisd rollAngle(M_PI, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(M_PI, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());//Coller a la conf du robot

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();

	//Build transformation matrix. Don't know what order i want, cf Kevin
	Eigen::Vector3d T(robot2BasePosition.x, robot2BasePosition.y, robot2BasePosition.z);
	Eigen::Matrix4d Trans; // Your Transformation Matrix
	Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	Trans.block<3,3>(0,0) = rotationMatrix;
	Trans.block<3,1>(0,3) = T;

	//Get the absolute pose using transformation matrix
	Eigen::Vector4d inputVector(unifiedPose.x, unifiedPose.y, unifiedPose.z, 1);
	Eigen::Vector4d resultHomogeneousVector = Trans.inverse()*inputVector;

	//Get the absolute orientation using rotation matrix
	Eigen::Quaternion<double> qUnifiedPose = RPYToQuaternion(unifiedPose);
		

	pose retval = quaternionToRPY(qUnifiedPose * q.conjugate());
	retval.x = resultHomogeneousVector(0);
	retval.y = resultHomogeneousVector(1); 
	retval.z = resultHomogeneousVector(2); 
	return retval;
}

pose quaternionToRPY(Eigen::Quaternion<double> q)
{
	pose retval;

	Eigen::Matrix3d rotationMatrix = q.matrix();
	//Eigen::Vector3d ea = rotationMatrix.eulerAngles(2, 1, 0);
	//Eigen::Vector3d ea = rotationMatrix.eulerAngles(2, 0, 1);
	//Eigen::Vector3d ea = rotationMatrix.eulerAngles(1, 0, 2);
	//Eigen::Vector3d ea = rotationMatrix.eulerAngles(1, 2, 0);
	Eigen::Vector3d ea = rotationMatrix.eulerAngles(0, 1, 2);//Works for Arm2
	//Eigen::Vector3d ea = rotationMatrix.eulerAngles(0, 2, 1);

	retval.roll = ea(0);
	retval.yaw = ea(1);
	retval.pitch = -ea(2);//Minus what i am expecting for arm 2	
	
	/*retval.roll = ea(0);
	retval.pitch = ea(1);
	retval.yaw = ea(2);*/

	return retval;	
}

Eigen::Quaternion<double> RPYToQuaternion(pose p)
{
	Eigen::AngleAxisd rollAngle(p.roll, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(-p.pitch, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(p.yaw, Eigen::Vector3d::UnitY());//Need to add a minus for arm2
	
	//Eigen::Quaternion<double> q = yawAngle * rollAngle * pitchAngle;//Nope
	//Eigen::Quaternion<double> q = pitchAngle * rollAngle * yawAngle;//Nope
	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;//This is the way
	//Eigen::Quaternion<double> q = yawAngle *  pitchAngle  *rollAngle;
	//Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;//Nope
	
	//Eigen::Quaternion<double> q = pitchAngle * yawAngle * rollAngle; //Nope
	//Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;//Nope


	return q;
}
