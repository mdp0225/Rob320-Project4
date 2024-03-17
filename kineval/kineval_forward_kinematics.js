|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
    kineval.buildFKTransforms();

}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

    function traverseFKBase() {
        //get the rotation matrix
        var r = robot.origin.rpy[0];
        var p = robot.origin.rpy[1];
        var y = robot.origin.rpy[2];
        var rotation_matrix = matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(y),generate_rotation_matrix_Y(p)),generate_rotation_matrix_X(r));
        
        //get the translation matrix
        var x = robot.origin.xyz[0];
        var y = robot.origin.xyz[1];
        var z = robot.origin.xyz[2];
        var translation_matrix = generate_translation_matrix(x,y,z);
        
        //updating the global heading variable
        var heading_vector = [[0],[0],[1],[1]];
        robot.links[robot.base].xform = matrix_multiply(translation_matrix,rotation_matrix);
        robot_heading = matrix_multiply(robot.links[robot.base].xform,heading_vector);
        
        //updating the global lateral variable
        var lateral_vector = [[1],[0],[0],[1]];
        robot_lateral = matrix_multiply(robot.links[robot.base].xform,lateral_vector);

        //converting coordinates if geometries are imported
        if (robot.links_geom_imported) {
            robot.links[robot.base].xform = matrix_multiply(robot.links[robot.base].xform, matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2)));
        }
    }
    
    function traverseFKLink(curr_link) {
        //set the xform of the current link
        robot.links[curr_link].xform = robot.joints[robot.links[curr_link].parent].xform;

        //traverse if  link has children
        if (robot.links[curr_link].children != undefined){
            //traverse through children
            var num_children = robot.links[curr_link].children.length;
            for (var i = 0; i < num_children; i++){
                var curr_child = robot.links[curr_link].children[i];
                traverseFKJoint(curr_child);
            }
        }
        else {
            return;
        }
    }
    
    function traverseFKJoint(curJoint) {
        //get the rotation matrix
        var r = robot.joints[curJoint].origin.rpy[0];
        var p = robot.joints[curJoint].origin.rpy[1];
        var y = robot.joints[curJoint].origin.rpy[2];
        var rotation_matrix = matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(y),generate_rotation_matrix_Y(p)),generate_rotation_matrix_X(r));

        //get the translation matrix
        var x = robot.joints[curJoint].origin.xyz[0];
        var y = robot.joints[curJoint].origin.xyz[1];
        var z = robot.joints[curJoint].origin.xyz[2];
        var translation_matrix = generate_translation_matrix(x,y,z);

        //get the joint transformation matrix
        var joint_matrix;
        if (robot.links_geom_imported){ 
            //if the joint is prismatic
            if (robot.joints[curJoint].type === "prismatic"){
                var angles = [ robot.joints[curJoint].angle, robot.joints[curJoint].angle, robot.joints[curJoint].angle];
                var v = robot.joints[curJoint].axis;
                var l = angles.length;
                var p = new Array(l);
                for (var i=0;i<l;++i){
                    p[i] = angles[i]*v[i];
                }
                angles = p;
                joint_matrix = generate_translation_matrix(angles[0],angles[1],angles[2]);

            //if the joint is continuous or revolute
            }else if (robot.joints[curJoint].type === "continuous"){
                //if the element in the axis is more than 0, make it 100
                if (robot.joints[curJoint].axis[0] > 0){
                    robot.joints[curJoint].axis[0] = 100;
                }
                if (robot.joints[curJoint].axis[1] > 0){
                    robot.joints[curJoint].axis[1] = 100;
                }
                if (robot.joints[curJoint].axis[2] > 0){
                    robot.joints[curJoint].axis[2] = 100;
                }
                joint_matrix = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(robot.joints[curJoint].axis,robot.joints[curJoint].angle)));
            }else if (robot.joints[curJoint].type === "revolute"){
                joint_matrix = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(robot.joints[curJoint].axis,robot.joints[curJoint].angle)));
            }else{
                joint_matrix = generate_identity(4);
            }
        }
        else{
            joint_matrix = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(robot.joints[curJoint].axis,robot.joints[curJoint].angle))); 
        }
        //get the final transformation matrix
        var transformation = matrix_multiply(robot.links[robot.joints[curJoint].parent].xform,matrix_multiply(translation_matrix,rotation_matrix));
        robot.joints[curJoint].xform = matrix_multiply(transformation, joint_matrix);

        //traverse through the link
        traverseFKLink(robot.joints[curJoint].child);
    }

    kineval.buildFKTransforms = function buildFKTransforms () {
        //determine where the base is
        traverseFKBase();

        //traverse through the child joints which will then traverse through the child links and so on
        var num_children = robot.links[robot.base].children.length;
        for (var  i = 0; i < num_children; i++){
            traverseFKJoint(robot.links[robot.base].children[i]);
        }
        
    } 
