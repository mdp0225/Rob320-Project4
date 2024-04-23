
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        console.log("index: " + kineval.motion_plan_traversal_index)
        console.log("here: " + kineval.motion_plan);
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}

function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

        // Initialize RRT trees if this is the first iteration
        if (rrt_iter_count === 0) {
            T_a = tree_init(q_start_config);
            T_b = tree_init(q_goal_config);
            rrt_iter_count++;
            return "searching";
        }

        // Implement single RRT iteration here
        if (rrt_alg === 1) { // RRT-Connect

            // Extend the trees towards random configurations
            var q_rand = random_config();

            // Extend tree T_a towards q_rand
            var extended_a = extendRRT(T_a, q_rand);
            console.log(extended_a);
            if (extended_a !== "trapped") {
                // Try connecting T_b to the new vertex in T_a
                var connected = connectRRT(T_b, T_a.vertices[T_a.newest].vertex);
                if (connected === "reached") {
                    // Path found
                    var path = createPath(T_a, T_b);
                    // Highlight vertices of found path
                    for (i = 0; i < path.length; i++) {
                        var pathTree = i < path.length / 2 ? T_a : T_b;
                        var pathIndex = i < path.length / 2 ? i : i - path.length / 2;
                        pathTree.vertices[pathIndex].geom.material.color = { r: 1, g: 0, b: 0 };
                        //path[i].geom.material.color = { r: 1, g: 0, b: 0 };
                    }
                    rrt_iterate = false;
                    return "reached";
                }
            }

            // Swap trees T_a and T_b
            var temp = T_a;
            T_a = T_b;
            T_b = temp;

            rrt_iter_count++;

            // Check if maximum iterations reached
            if (rrt_iter_count >= 1000) {
                rrt_iterate = false;
                return "failed";
            }

            return "searching";
        }
    }
    return "searching";
}


//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs

    // Generate a random configuration within the bounds of the planning space
function random_config() {
    return [
        Math.random() * (robot_boundary[1][1] - robot_boundary[0][0]) + robot_boundary[0][0],
        Math.random() * (robot_boundary[1][1] - robot_boundary[0][0]) + robot_boundary[0][0],
        Math.random() * (robot_boundary[1][1] - robot_boundary[0][0]) + robot_boundary[0][0]
    ];
}

// Generate a new configuration by extending the tree towards a random configuration
function newConfig(q_from, q_to) {
    var eps = 0.5;
    var dx = q_to[0] - q_from[0];
    var dy = q_to[1] - q_from[1];
    var dz = q_to[2] - q_from[2];
    var dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

    if (dist <= eps) {
        return q_to;
    } else {
        return [
            q_from[0] + eps * dx / dist,
            q_from[1] + eps * dy / dist,
            q_from[2] + eps * dz / dist
        ];
    }
}

// Find the nearest neighbor in the RRT tree to a given configuration
function findNearestNeighbor(tree, q) {
    var minDist = Number.MAX_VALUE;
    var nearestIdx = -1;

    for (var i = 0; i < tree.vertices.length; i++) {
        var dist = distance(tree.vertices[i].vertex, q);
        if (dist < minDist) {
            minDist = dist;
            nearestIdx = i;
        }
    }
    return nearestIdx;
}

// Extend the RRT tree towards a random configuration
function extendRRT(tree, target) {
    var eps = 0.5;
    var q_near_idx = findNearestNeighbor(tree, target);
    var q_near = tree.vertices[q_near_idx].vertex;
    var q_new = newConfig(q_near, target);

    var collision_result = kineval.poseIsCollision(q_new);
    console.log("iteration: " + rrt_iter_count + "collision: " + collision_result);
    if (!collision_result) {
        tree_add_vertex(tree, q_new);
        tree_add_edge(tree, q_near_idx, tree.newest);

        if (distance(q_new, target) <= eps) {
            return "reached";
        }
        return "advanced";
    }
    return "trapped";
}

// Connect the two RRT trees
function connectRRT(tree, q_target) {
    var extended = extendRRT(tree, q_target);
    while (extended === "advanced") {
        extended = extendRRT(tree, q_target);
    }
    return extended;
}

function distance(q1, q2) {
    var dx = q2[0] - q1[0];
    var dy = q2[1] - q1[1];
    return Math.sqrt(dx * dx + dy * dy);
}

// Create a path from the start configuration to the goal configuration
function createPath(tree1, tree2) {
    var path = [];
    var current_vertex = tree1.vertices[tree1.newest];
    while (current_vertex !== undefined) {
        path.unshift(current_vertex.vertex);
        current_vertex = current_vertex.parent;
    }
    current_vertex = tree2.vertices[tree2.newest];
    while (current_vertex !== undefined) {
        path.unshift(current_vertex.vertex);
        current_vertex = current_vertex.parent;
    }
    kineval.motion_plan = path;
    return path;
}

