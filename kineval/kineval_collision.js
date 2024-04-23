
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | collision detection

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

// KE: merge collision test into FK
// KE: make FK for a configuration and independent of current robot state

kineval.robotIsCollision = function robot_iscollision() {
    // test whether geometry of current configuration of robot is in collision with planning world 

    // form configuration from base location and joint angles
    var q_robot_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_robot_config.length;
        q_robot_config = q_robot_config.concat(robot.joints[x].angle);
    }

    // test for collision and change base color based on the result
    collision_result = kineval.poseIsCollision(q_robot_config);

    robot.collision = collision_result;
}


kineval.poseIsCollision = function robot_collision_test(q) {
    // perform collision test of robot geometry against planning world 

    // test base origin (not extents) against world boundary extents
    if ((q[0]<robot_boundary[0][0])||(q[0]>robot_boundary[1][0])||(q[2]<robot_boundary[0][2])||(q[2]>robot_boundary[1][2]))
        return robot.base;

    // traverse robot kinematics to test each body for collision
    // STENCIL: implement forward kinematics for collision detection
    //console.log("collision name: " + robot_collision_forward_kinematics(q));
    return robot_collision_forward_kinematics(q);


}

function robot_collision_forward_kinematics(q) {
    // Start traversal from the base link
    return traverse_collision_forward_kinematics_link(robot.links[robot.base], generate_identity(4), q);  
}

function traverse_collision_forward_kinematics_link(link,mstack,q) {

    /* test collision FK
    console.log(link);
    */
    if (typeof link.visual !== 'undefined') {
        var local_link_xform = matrix_multiply(mstack,generate_translation_matrix(link.visual.origin.xyz[0],link.visual.origin.xyz[1],link.visual.origin.xyz[2]));
    }
    else {
        var local_link_xform = matrix_multiply(mstack,generate_identity());
    }

    // test collision by transforming obstacles in world to link space
/*
    mstack_inv = matrix_invert_affine(mstack);
*/
    mstack_inv = numeric.inv(mstack);

    var i;
    var j;

    // test each obstacle against link bbox geometry by transforming obstacle into link frame and testing against axis aligned bounding box
    //for (j=0;j<robot_obstacles.length;j++) { 
    for (j in robot_obstacles) { 

        var obstacle_local = matrix_multiply(mstack_inv,robot_obstacles[j].location);

        // assume link is in collision as default
        var in_collision = true; 

        // if obstacle lies outside the link extents along any dimension, no collision is detected
        if (
            (obstacle_local[0][0]<(link.bbox.min.x-robot_obstacles[j].radius))
            ||
            (obstacle_local[0][0]>(link.bbox.max.x+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[1][0]<(link.bbox.min.y-robot_obstacles[j].radius))
            ||
            (obstacle_local[1][0]>(link.bbox.max.y+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[2][0]<(link.bbox.min.z-robot_obstacles[j].radius)) 
            ||
            (obstacle_local[2][0]>(link.bbox.max.z+robot_obstacles[j].radius))
        )
                in_collision = false;

        // if obstacle lies within link extents along all dimensions, a collision is detected and return true
        if (in_collision)
            return link.name;
    }

    // recurse child joints for collisions, returning true if child returns collision
    if (typeof link.children !== 'undefined') { // return if there are no children
        var local_collision;
        //for (i=0;i<link.children.length;i++) {
        for (i in link.children) {
            local_collision = traverse_collision_forward_kinematics_joint(robot.joints[link.children[i]],mstack,q)
            if (local_collision)
                return local_collision;
        }
    }

    // return false, when no collision detected for this link and children 
    return false;
}

function traverse_collision_forward_kinematics_joint(joint, mstack, q) {
    // Get joint transformation matrix based on joint type and angle
    var joint_matrix;
    if (robot.links_geom_imported) {
        if (joint.type === "prismatic") {
            // Implement translation for prismatic joint
            // joint_matrix = generate_translation_matrix(q[q_names[joint.name]], q[q_names[joint.name] + 1], q[q_names[joint.name] + 2]);
            var angles = [ joint.angle,joint.angle, joint.angle];
            var v = joint.axis;
            var l = angles.length;
            var p = new Array(l);
            for (var i=0;i<l;++i){
                p[i] = angles[i]*v[i];
            }
            angles = p;
            joint_matrix = generate_translation_matrix(angles[0],angles[1],angles[2]);
        } 
        else if (joint.type === "continuous" || joint.type === "revolute") {
            // Implement rotation for continuous or revolute joint
            //joint_matrix = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(joint.axis, q[q_names[joint.name]])));
            joint_matrix = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(joint.axis, joint.angle)));
        } 
        else {
            joint_matrix = generate_identity(4);
        }
    } 
    else {
        //joint_matrix = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(joint.axis, q[q_names[joint.name]])));
        joint_matrix = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(joint.axis, joint.angle)));
    }

    // Calculate joint transformation matrix
    var transformation = matrix_multiply(robot.links[joint.parent].xform, joint_matrix);

    // Update matrix stack
    var new_mstack = matrix_multiply(transformation, mstack);

    // Recurse through the link connected to this joint
    return traverse_collision_forward_kinematics_link(robot.links[joint.child], new_mstack, q);
}


