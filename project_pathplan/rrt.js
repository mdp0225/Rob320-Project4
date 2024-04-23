/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}

function iterateRRTConnect() {
    // Extend the RRT tree from the start configuration to the nearest vertex in the other tree

    var q_rand = randomConfig();
    var extended = extendRRT(T_a, q_rand);
    if (extended !== "trapped") {
        // Connect the two RRT trees 
        var connected = connectRRT(T_b, T_a.vertices[T_a.newest].vertex);

        if (connected === "reached") {
            // Path found
            var path = createPath(T_a, T_b);
            //console.log(path);
            drawHighlightedPath(path);
            search_iterate = false;
            return "succeeded";
        }
    }

    // Swap trees
    var temp = T_a;
    T_a = T_b;
    T_b = temp;

    // next iteration
    search_iter_count++;

    // if max iter
    if (search_iter_count >= search_max_iterations) {
        search_iterate = false;
        return "failed";
    }

    //otherwise keep going
    return "extended";
}

function iterateRRTStar() {

}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath

// Extend RRT tree towards a random configuration
function extendRRT(tree, target) {
    
    //var q_rand = randomConfig();

    // Find the nearest vertex in the tree to q_rand
    var q_near_idx = findNearestNeighbor(tree, target);
    var q_near = tree.vertices[q_near_idx].vertex;

    // Generate a new configuration by extending the tree towards q_rand
    var q_new = newConfig(q_near, target);
    search_visited++;

    // Check if q_new is in collision
    if (!testCollision(q_new)) {
        // Insert q_new into the tree
        insertTreeVertex(tree, q_new);

        // Add an edge between q_near and q_new
        insertTreeEdge(tree, q_near_idx, tree.newest);

        // Check if q_new is close enough to q_goal
        var dx = q_new[0] - target[0];
        var dy = q_new[1] - target[1];
        if (Math.abs(dx) <= eps && Math.abs(dy) <= eps){
            return "reached";
        }
    return "advanced";
    }
    return "trapped";
}

// Connect the two RRT trees
function connectRRT(tree, q_target) {
    // Extend the tree towards the target configuration
    var extended = extendRRT(tree, q_target);
    while (extended === "advanced"){
        extended = extendRRT(tree, q_target);
    }
    return extended;
}

// Generate a random configuration within the bounds of the planning space
function randomConfig() {
    return [Math.random() * 7 - 1.8, Math.random() * 7 - 1.8];
}

// Generate a new configuration by extending the tree towards a random configuration
function newConfig(q_from, q_to) {
    var dx = q_to[0] - q_from[0];
    var dy = q_to[1] - q_from[1];
    var dist = Math.sqrt(dx * dx + dy * dy);

    if (Math.abs(dx) <= eps && Math.abs(dy) <= eps) {
        return q_to;
    } 
    else {
        return [q_from[0] + eps * dx / dist, q_from[1] + eps * dy / dist];
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
    //var nearestNeighbor = tree.vertices[nearestIdx].vertex;
    return nearestIdx;
}

// Helper function to compute the Euclidean distance between two configurations
function distance(q1, q2) {
    var dx = q2[0] - q1[0];
    var dy = q2[1] - q1[1];
    return Math.sqrt(dx * dx + dy * dy);
}

//find a path which is an array of vertices from start to goal
function createPath(tree1, tree2) {
    // Initialize the path tree
    var path = [];

    // Start from the goal configuration in tree1 and trace back to the root
    var current_vertex = tree1.vertices[tree1.newest];
    console.log("tree1 " + tree1.vertices[tree1.newest].vertex);
    while (current_vertex !== undefined) {
        path.unshift(current_vertex.vertex);
        console.log("currVer " + current_vertex.vertex);
        console.log("currPar " + current_vertex.parent);
        if (current_vertex.parent !== undefined) {
            current_vertex = current_vertex.parent;
        } 
        else {
            current_vertex = undefined;
        }
    }
    

    // Append the path from the root to the goal configuration in tree2
    current_vertex = tree2.vertices[tree2.newest];
    console.log("tree2 " + tree2.vertices[tree2.newest].vertex);
    console.log("length " + tree2.vertices.length);
    while (current_vertex !== undefined) {
        path.unshift(current_vertex.vertex);
        console.log("currVer2 " + current_vertex.vertex);
        console.log("currPar2 " + current_vertex.parent);
        if (current_vertex.parent !== undefined) {
            current_vertex = current_vertex.parent;
        } 
        else {
            current_vertex = undefined;
        }
    }
    

    return path;
}

