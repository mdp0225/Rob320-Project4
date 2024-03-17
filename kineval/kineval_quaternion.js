//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

// STENCIL: reference quaternion code has the following functions:
//   quaternion_from_axisangle
//   quaternion_normalize
//   quaternion_to_rotation_matrix
//   quaternion_multiply

// **** Function stencils are provided below, please uncomment and implement them ****//

function quaternion_from_axisangle(axis, angle) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = [ Math.cos(angle/2), axis[0]*Math.sin(angle/2), axis[1]*Math.sin(angle/2), axis[2]*Math.sin(angle/2)];
    return q;
}

function quaternion_normalize(q1){
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = [];
    var elem = 0;
    q.length = q1.length;
    for (var i = 0; i < q1.length; i++){
        elem = elem + q1[i]*q1[i];
    }
    elem = Math.sqrt(elem);
    for (var i = 0; i < q1.length; i++){
        if (Math.abs(elem)<(Math.exp(10,-16))){
            q[i] = q1[i];
        } 
        else {
            q[i]=q1[i]/elem;
        }
    }
    return q;

}

kineval.quaternionMultiply = function quaternion_multiply(q1,q2) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = [];
    q.length = q1.length;
    q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
    return q;
}

function quaternion_to_rotation_matrix(q){
    // returns 4-by-4 2D rotation matrix
    var mat = generate_identity(4);
    mat[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    mat[0][1] = 2*( q[1]*q[2] - q[0]*q[3] );
    mat[0][2] = 2*( q[0]*q[2] + q[1]*q[3] );
    mat[1][0] = 2*( q[1]*q[2] + q[0]*q[3] );
    mat[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    mat[1][2] = 2*( q[3]*q[2] - q[0]*q[1] );
    mat[2][0] = 2*( q[1]*q[3] - q[0]*q[2] );
    mat[2][1] = 2*( q[1]*q[0] + q[2]*q[3] );
    mat[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    return mat;
}

