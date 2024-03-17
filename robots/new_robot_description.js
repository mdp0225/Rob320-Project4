//   CREATE ROBOT STRUCTURE

// KE 

links_geom_imported = false;

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "new_robot_description";

//partner name
robot.partner_name = "dliujm";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0,0], rpy:[0,0,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "torso";  

        
// specify and create data objects for the links of the robot
robot.links = {
    "torso": {},   
    "left_leg": {}, 
    "right_leg": {} , 
    "left_arm": {}, 
    "right_arm": {}, 
    "head": {}
};
/* for you to do
, "shoulder_left": {}  , "upperarm_left": {} , "forearm_left": {} };
*/

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.right_leg = {parent:"torso", child:"right_leg"};
robot.joints.right_leg.origin = {xyz: [0.3,0.8,0.0], rpy:[-Math.PI/2,0,0]};
robot.joints.right_leg.axis = [7, 0, 0]; 

robot.joints.left_leg = {parent:"torso", child:"left_leg"};
robot.joints.left_leg.origin = {xyz: [-0.3,0.8,0], rpy:[Math.PI/2,0,0]};
robot.joints.left_leg.axis = [7, 0, 0]; 

robot.joints.right_arm = {parent:"torso", child:"right_arm"};
robot.joints.right_arm.origin = {xyz: [0.4,1.8,0], rpy:[1,0,0]};
robot.joints.right_arm.axis = [7, 0, 0]; 

robot.joints.left_arm = {parent:"torso", child:"left_arm"};
robot.joints.left_arm.origin = {xyz: [-0.4,1.8,0], rpy:[1,0,0]};
robot.joints.left_arm.axis = [7, 0, 0]; 

robot.joints.head = {parent:"torso", child:"head"};
robot.joints.head.origin = {xyz: [0,1.8,0], rpy:[0,0,0]};
robot.joints.head.axis = [0, 5, 0]; 

//specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "left_arm";
robot.endeffector.position = [[1],[1],[0.5],[1]]

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};

links_geom["torso"] = new THREE.CubeGeometry( 0.8, 1.2, 0.4 );
links_geom["torso"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 1.3, 0) );

links_geom["right_arm"] = new THREE.CubeGeometry( 0.2, 0.2, 0.8 );
links_geom["right_arm"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["left_arm"] = new THREE.CubeGeometry( 0.2, 0.2, 0.8 );
links_geom["left_arm"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["right_leg"] = new THREE.CubeGeometry( 0.3, 0.3, 0.8 );
links_geom["right_leg"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, -0.4) );

links_geom["left_leg"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["left_leg"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.4) );

links_geom["head"] = new THREE.CubeGeometry( 0.4, 0.4, 0.4 );
links_geom["head"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.25, 0) );

