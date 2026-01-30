if(!("Entities" in this)) return;

const a_ = 56.0;
const a2 = 3136.0
const b_ = 32.0;
const b2 = 1024.0;
const a2mb2 = 2112.0;
const a2pb2 = 4160.0;
const atb = 1792.0;
const a2tb2 = 3211264.0;

// Precalculated constants
const d1x = 151484.09686731211638346807701780282673155084349221841210723666217;
const d1y = 65259.463559493120080357854500977681313126585891064972886646114392;
const d1z = 45610.448033096360092265308747487004729466142379513813304004751523;

// Ordinary math functions

function min(a,b) { return a>b ? b : a; }

function Vector_Eq(a, b) {
	return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}

function Matrix_Eq(A, B) {
	local result = true;
	for(local i=0;i<3;i++){
		for(local j=0;j<3;j++){
			result = result && (A[i][j] == B[i][j]);
		};
	};
	return result;
}

function Vectors_To_Matrix(ex, ey, ez) {
	return [[ex.x, ey.x, ez.x], [ex.y, ey.y, ez.y], [ex.z, ey.z, ez.z]];
}

function Vectors_To_Matrix_Inv(ex, ey, ez) {
	return [[ex.x, ex.y, ex.z], [ey.x, ey.y, ey.z], [ez.x, ez.y, ez.z]];
}

function Matrix_Mult_3D(A, B){
	result <- [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
	local sum = 0.0; 
	for(local i=0;i<3;i++){
		for(local j=0;j<3;j++){
			for(local k=0;k<3;k++){
				sum += A[i][k] * B[k][j];
			};
			result[i][j] = sum;
			sum = 0.0;
		};
	};
	return result;
};

function Matrix_Vector_Mult_3D(A, b){
	return Vector(A[0][0]*b.x + A[0][1]*b.y + A[0][2]*b.z, A[1][0]*b.x + A[1][1]*b.y + A[1][2]*b.z, A[2][0]*b.x + A[2][1]*b.y + A[2][2]*b.z);
};

function Solve_2D_System(A, b) {
	local d = A[0][0]*A[1][1] - A[1][0]*A[0][1];
	
	if (d != 0.0) {
		local d_p = [b[0]*A[1][1] - A[1][0]*b[1], A[0][0]*b[1] - b[0]*A[0][1]];
		return [d_p[0]/d, d_p[1]/d];
	}

	if (A[0][0] + A[0][1] != 0.0)
		return [ b[0] / (A[0][0] + A[0][1]), b[0] / (A[0][0] + A[0][1])];

	if (A[1][0] + A[1][1] != 0.0)
		return [ b[1] / (A[1][0] + A[1][1]), b[1] / (A[1][0] + A[1][1])];

	return [0.0, 0.0]

}

// Coordinate transformations

function Get_Ellipsoidal_Coords(vec) {
	local w1 = vec.LengthSqr() - a2pb2;
	local w2 = b2*vec.x*vec.x + a2*vec.y*vec.y + a2pb2*vec.z*vec.z - a2tb2;
	local w3 = a2tb2*vec.z*vec.z;
	
	local P = (w1*w1 + 3*w2)/9;
	local Q = (2*w1*w1*w1 + 9*w1*w2 + 27*w3)/54;
	
	local R = Q/(P*sqrt(P));
	if (R>1) R=1.0;
	if (R<(-1)) R=-1.0;
	
	local alpha = acos(R);
	
	local k2 = 2*sqrt(P)*cos(alpha/3) + w1/3;
	local u2 = -(2*sqrt(P)*cos(alpha/3 - 2*PI/3) + w1/3);
	local v2 = -(2*sqrt(P)*cos(alpha/3 + 2*PI/3) + w1/3);
	
	if(k2<0) k2=0;
	if(u2<0) u2=0;
	if(u2>b2) u2=b2;
	if(v2<b2) v2=b2;
	if(v2>a2) v2=a2;
	
	return Vector(sqrt(k2), sqrt(u2), sqrt(v2));
};

// Integrands

function E_0(x) {
	local R = a2mb2*(cos(x)*cos(x)) + b2
	return 1.0/sqrt(R);
};

function E_1x(x) {
	local R = a2mb2*(cos(x)*cos(x)) + b2
	return (cos(x)*cos(x))/(R*sqrt(R));
};

function E_1y(x) {
	local R = a2mb2*(cos(x)*cos(x)) + b2
	return (cos(x)*cos(x))/(b2*sqrt(R));
};

function E_1z(x) {
	local R1 = a2mb2*(cos(x)*cos(x)) + b2
	local R2 = a2*(cos(x)*cos(x)) + b2
	return R2/(b2*(a2*sqrt(R1)*(cos(x)*cos(x)) + a_*R1));
};

// Integration function
// Uses Simpson's rule

function Integral(lb, rb, func, n) {
	local h = (rb - lb)/n;
	local s = func(lb) + func(rb)
	
	for(local i=1; i<n; i++) {
		s += (2*(i/2)==i ? 2 : 4) * func(i*h + lb);
	}
	
	return s*h/3;
}

// Auxiliary vector
// Vector of a unit change in first coordinate in the elliptical coordinate system
// transformed back into Cartesian coordinates

function Get_Aux_Vec(loc, elip) {
	
	local v1 = loc.x*elip.x / (a2 + elip.x*elip.x);
	local v2 = loc.y*elip.x / (b2 + elip.x*elip.x);
	local v3 = loc.z > 0 ? (elip.y*elip.z / atb) : (-elip.y*elip.z / atb);
	
	return Vector(v1, v2, v3);
};

// Basis accelerations

function Get_g0_Vec(loc, elip, aux) {
local Ra = (a2 + elip.x*elip.x);
	local Rb = (b2 + elip.x*elip.x);
	return aux * (1.0/(aux.LengthSqr()*sqrt(Ra*Rb)));
};

function Get_g1x_Vec(loc, elip, aux) {
	local Ra = (a2 + elip.x*elip.x);
	local Rb = (b2 + elip.x*elip.x);
	
	local v1 = Vector(-Integral(atan(elip.x/b_), PI/2, E_1x, 16), 0.0, 0.0);
	local v2 = aux * (loc.x /(aux.LengthSqr()*Ra*sqrt(Ra*Rb)));
	return (v1 + v2)*d1x*0.5;
};

function Get_g1y_Vec(loc, elip, aux) {
	local Ra = (a2 + elip.x*elip.x);
	local Rb = (b2 + elip.x*elip.x);
	
	local v1 = Vector(0.0, -Integral(atan(elip.x/b_), PI/2, E_1y, 16), 0.0);
	local v2 = aux * (loc.y /(aux.LengthSqr()*Rb*sqrt(Ra*Rb)));
	return (v1 + v2)*d1y*0.5;
};

function Get_g1z_Vec(loc, elip, aux) {
	local aux2 = Vector(loc.x / (a2 + elip.x*elip.x), loc.y / (b2 + elip.x*elip.x), 0.0);
	local Ra = (a2 + elip.x*elip.x);
	local Rb = (b2 + elip.x*elip.x);
	
	local v1 = (aux2 * -aux.z + Vector(0.0, 0.0, elip.x * aux2.LengthSqr())) * (1/(aux.LengthSqr()*atb))
	local v2 = Vector(0.0, 0.0, -Integral(atan(elip.x/b_), PI/2, E_1z, 16))
	local v3 = aux * ((elip.x*elip.x + a2pb2) /(aux.LengthSqr()*(a2tb2*sqrt(Ra*Rb) + atb*Ra*Rb)))
	return (v1 + v2 + v3)*d1z*0.5;
};
