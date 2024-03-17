//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//


function  matrix_multiply(m1,m2){
    // returns 2D array that is the result of m1*m2
    var m = new Array(m1.length);
    for (i=0;i<m1.length;++i){
        m[i] = new Array(m2[0].length);
        for (j=0;j<m2[0].length;++j){
            var val = 0;
            for (k=0;k<m2.length;++k){
                val = val + m1[i][k]*m2[k][j];

            }
            m[i][j]=val;
        }
    }
    return m;
}

function matrix_transpose(m) {
    // returns 2D array that is the result of m1*m2  <--- don't know if this comment is correct?
    var result = [];
    for (var i = 0; i < m[0].length; i++) {
        result[i] = [];
        for (var j = 0; j < m.length; j++) {
            result[i][j] = m[j][i];
        }
    }
    return result;
}

// function matrix_pseudoinverse(m) {
//     // returns pseudoinverse of matrix m

// }

// function matrix_invert_affine(m) {
//     // returns 2D array that is the invert affine of 4-by-4 matrix m

// }

function vector_normalize(v) {
    // returns normalized vector for v
    var divisor = 0;
    for (var i = 0; i < v.length; i++) {
        divisor += v[i] * v[i];
    }
    divisor = Math.sqrt(divisor);
    for (var i = 0; i < v.length; i++) {
        v[i] = v[i] / divisor;
    }
    return v;
}

function vector_cross(a,b) {
    // return cross product of vector a and b with both has 3 dimensions
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ];
}

function generate_identity() {
    // returns 4-by-4 2D array of identity matrix
    return [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ];
}

function generate_translation_matrix(tx, ty, tz) {
    // returns 4-by-4 matrix as a 2D array
    return [
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ];
}

function generate_rotation_matrix_X(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var cos = Math.cos(angle);
    var sin = Math.sin(angle);
    return [
        [1, 0, 0, 0],
        [0, cos, -sin, 0],
        [0, sin, cos, 0],
        [0, 0, 0, 1]
    ];
}

function generate_rotation_matrix_Y(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var cos = Math.cos(angle);
    var sin = Math.sin(angle);
    return [
        [cos, 0, sin, 0],
        [0, 1, 0, 0],
        [-sin, 0, cos, 0],
        [0, 0, 0, 1]
    ];
}

function generate_rotation_matrix_Z(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var cos = Math.cos(angle);
    var sin = Math.sin(angle);
    return [
        [cos, -sin, 0, 0],
        [sin, cos, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ];
}
