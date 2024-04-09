/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/


kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code

    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
        + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
        + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
    kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
    kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
    kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
    kineval.params.trial_ik_random.targets += 1;
    textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
    robot.jacobian = [];
    robot.dq = [];
    robot.dx = [];
    var curr_joint = endeffector_joint;
    var joint_index = 0;
    var more_joints = true;
    
    while (more_joints){
        // create world pose

        var endeffector_position_world = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
        var endeffector_position_world_temp = [];
        for (var i = 0; i< 3; i++){
            endeffector_position_world_temp[i] = [endeffector_position_world[i][0]];
        }
        
        // calculate error (robot.dx)
        for (var i = 0; i< 3; i++){
            robot.dx[i] = [endeffector_target_world.position[i] - endeffector_position_world_temp[i]];
        }
        for (i = 3; i< 6; i++){
            robot.dx[i] = [0];
        }

        // calculate jacobian (robot.jacobian)
        var ax = robot.joints[curr_joint].axis; 
        ax[3] = 1;
        
        console.log("ax" + ax);
        var k = matrix_multiply(robot.joints[curr_joint].xform, ax);
        console.log(k);
        k.pop();
        var kw = [];
        for (var i = 0; i< 3; i++){
            kw[i] = [k[i][0]];
        }
        console.log("kw" + kw[0] + kw[1] + kw[2]);
        console.log(robot.joints[curr_joint].xform);
        var origin = matrix_multiply(robot.joints[curr_joint].xform, [[0],[0],[0],[1]]);
        var origin = [origin[0][0], origin[1][0], origin[2][0]];

        console.log(k);
        console.log("origin" + origin);
        
        var jv1  = vector_subtraction(k,origin)

        console.log("jv1" + jv1);

        var jv2 = vector_subtraction(endeffector_position_world_temp,origin)
        var cross = vector_cross(jv1,jv2);
        var jw = vector_subtraction(k, origin);

        // set jacobian
        robot.jacobian[joint_index] = [cross[0], cross[1], cross[2],jw[0],jw[1],jw[2]];

       
        joint_index++;
        // check if no more joints
        if (robot.joints[curr_joint].parent === robot.base){
            more_joints = false;
        }

        // move to next joint
        curr_joint = robot.links[robot.joints[curr_joint].parent].parent;
    }

    robot.jacobian = matrix_transpose(robot.jacobian);
    if (!kineval.params.ik_pseudoinverse){
        robot.dq = matrix_multiply(matrix_transpose(robot.jacobian),robot.dx);
    } 
    else {
        robot.dq = matrix_multiply(matrix_pseudoinverse(robot.jacobian), robot.dx);
    }

    curr_joint2 = endeffector_joint;
    joint_index2 = 0;
    more_joints2 = true;
    while (more_joints2){
        robot.joints[curr_joint2].control = kineval.params.ik_steplength * robot.dq[joint_index2][0];

        joint_index2++;
        if (robot.joints[curr_joint2].parent === robot.base){
            more_joints2 = false;
        }
        curr_joint2 = robot.links[robot.joints[curr_joint2].parent].parent;
    }
    
    
}

function vector_subtraction(v1, v2) {
    if (v1.length !== v2.length) {
        throw new Error("Vector dimensions do not match.");
    }

    var result = [];
    for (var i = 0; i < v1.length; i++) {
        result.push(v1[i] - v2[i]);
    }
    return result;
}
