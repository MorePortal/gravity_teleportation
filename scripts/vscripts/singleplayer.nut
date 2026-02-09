if(!("Entities" in this)) return;
IncludeScript("maths");

names <- ["prop_weighted_cube", "prop_monster_box", "prop_paint_bomb", "npc_portal_turret_floor", "npc_security_camera", "prop_exploding_futbol"];
pushes <- [];
props <- [];
all_portals <- [];
detectors <- [];
detected_blue <- null;
detected_orange <- null;

portal <- [null, null];
portal_temp <- [null, null];
origin <- [null, null];
local_matrix <- [null, null,];
local_inv <- [null, null];
local_g <- [null, null];
constants <- [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]];

// Physical properties
const dt = 0.0333333333333333333333333333333333333333333;
const g = 600.0;

// Number of props which we are tracking
const max_props = 4;

// Integrals' precision (amount of intervals the range of integration is divided into)
const precision = 16

// Portal potential and it's derivatives induced by itself
self_potential <- [[0.0, 0.0, 0.0, 0.0]]
self_potential.append([0.0, -Integral(0.0, PI/2, E_1x, precision), 0.0, 0.0])
self_potential.append([0.0, 0.0, -Integral(0.0, PI/2, E_1y, precision), 0.0])
self_potential.append([0.0, 0.0, 0.0, -Integral(0.0, PI/2, E_1z, precision)])
// Technically, there are self_potential[0][3] and self_potential[3][0] terms, but they will cancel themselves out

// Variables to draw a field
show_mins_glob <- Vector(0.0, 0.0, 0.0)
show_maxs_glob <- Vector(0.0, 0.0, 0.0)
calc_mins_glob <- Vector(0.0, 0.0, 0.0)
calc_maxs_glob <- Vector(0.0, 0.0, 0.0)
divx_glob <- 1
divy_glob <- 1
side_glob <- 1
err_term_glob <- 0.000004
redraw <- 0;
pos <- [0, 0, 0, null]
stepsize <- 0.0
line_sign <- 0
inbounds <- 2;
Orange <- [255, 154, 0]
Blue <- [39, 167, 216]
gamma <- 2.2

// Constants for RKF45 ODE solver
RKF45 <- [[0, 0, 0, 0, 0], [1.0/4.0, 0, 0, 0, 0], [3.0/32.0, 9.0/32.0, 0, 0, 0], [1932.0/2197.0, -7200.0/2197.0, 7296.0/2197.0, 0, 0], [439.0/216.0, -8, 3680.0/513.0, -845.0/4104.0, 0], [-8.0/27.0, 2, -3544.0/2565.0, 1859.0/4104.0, -11.0/40.0]]
RKF4 <- [25.0/216.0, 0, 1408.0/2565.0, 2197.0/4104.0, -1.0/5.0]
RKFerr <- [16.0/135.0 - 25.0/216.0, 0, 6656.0/12825.0 - 1408.0/2565.0, 28561.0/56430.0 - 2197.0/4104.0, -9.0/50.0 + 1.0/5.0, 2.0/55.0]

main_loop <- function() {
	
	// Calculates and applies accelerations

	Run_Physics <- function() {
		
			for (local i=-1; i<props.len(); i++) {
			
				if (i < 0)
					curr <- GetPlayer();
				else
					curr <- props[i];
					
				if(!curr) continue;
				if(!curr.IsValid()) continue;
				
				if(Check_Funnels(curr)) continue;
				
				acceleration <- Vector(0.0, 0.0, 0.0);
				
				for(local j=0; j<2; j++) {
					
					local_v <- Matrix_Vector_Mult_3D(local_inv[j], curr.GetCenter() - origin[j]);
					ellipt_v <- Get_Ellipsoidal_Coords(local_v);
					aux_v <- Get_Aux_Vector(local_v, ellipt_v);
					
					if (aux_v.LengthSqr() == 0.0) continue;
					
					g0 <- Get_g0(ellipt_v, aux_v) * constants[j][0]
					g1x <- Get_g1x(local_v, ellipt_v, aux_v, precision) * constants[j][1]
					g1y <- Get_g1y(local_v, ellipt_v, aux_v, precision) * constants[j][2]
					g1z <- Get_g1z(local_v, ellipt_v, aux_v, precision) * constants[j][3]
					
					acceleration += Matrix_Vector_Mult_3D(local_matrix[j], (g0 + g1x + g1y + g1z)*-1);
				}
				
				if(i < 0) {
					curr.SetVelocity(curr.GetVelocity() + acceleration*dt);
					continue;
				}

				dir <- acceleration;
				pushes[i].__KeyValueFromFloat("magnitude", dir.Norm() / 50); // I don't know why you need to divide by fifty (source code I found says that it multiplies by 100), but it works
				pushes[i].SetOrigin(curr.GetOrigin());
				pushes[i].SetForwardVector(dir);			
			}
			return;
		}
		
	// Funnels should nullify portalled gravity too
	
	function Check_Funnels(object) {
		is_colliding <- false;
		
		object_origin <- object.GetOrigin()
		object_normals <- [object.GetForwardVector(), object.GetLeftVector(), object.GetUpVector()]
		object_local <- Vectors_To_Matrix(object_normals[0], object_normals[1], object_normals[2])
		object_bbx <- [object.GetBoundingMins(), object.GetBoundingMaxs()]
		object_verticies <- [];
		foreach(i in [0,1])
			foreach(j in [0,1])
				foreach(k in [0,1])
					object_verticies.append(Matrix_Vector_Mult_3D(object_local, Vector(object_bbx[i].x, object_bbx[j].y, object_bbx[k].z)) + object_origin)
		
		funnel <- null;
		while (funnel = Entities.FindByClassname(funnel, "trigger_tractorbeam")) {
			is_colliding_single <- true;
			
			funnel_origin <- funnel.GetOrigin();
			funnel_normals <- [funnel.GetForwardVector(), funnel.GetLeftVector(), funnel.GetUpVector()]
			funnel_local <- Vectors_To_Matrix(funnel_normals[0], funnel_normals[1], funnel_normals[2])
			funnel_bbx <- [funnel.GetBoundingMins(), funnel.GetBoundingMaxs()];
			funnel_verticies <- [];
			foreach(i in [0,1])
				foreach(j in [0,1])
					foreach(k in [0,1])
						funnel_verticies.append(Matrix_Vector_Mult_3D(funnel_local, Vector(funnel_bbx[i].x, funnel_bbx[j].y, funnel_bbx[k].z)) + funnel_origin)
			axes <- []
			axes.extend(object_normals)
			axes.extend(funnel_normals)
			for (local i = 0; i<3; i++)
				for (local j = 0; j<3; j++)
					axes.append(object_normals[i].Cross(funnel_normals[j]))
			
			for(local i = 0; i<15; i++) {
				object_min <-  999999;
				object_max <- -999999;
				funnel_min <-  999999;
				funnel_max <- -999999;
				for(local j = 0; j<8; j++) {
					dot <- object_verticies[j].Dot(axes[i])
					if (dot > object_max)
						object_max <- dot
					if (dot < object_min)
						object_min <- dot
						
					dot <- funnel_verticies[j].Dot(axes[i])
					if (dot > funnel_max)
						funnel_max <- dot
					if (dot < funnel_min)
						funnel_min <- dot
				}
				
				if ((funnel_max < object_min) || (object_max < funnel_min)) {
					is_colliding_single <- false;
					break;
				}

			}
			if (is_colliding_single) {
				is_colliding <- true;
				break;
			}
		}
		
		return is_colliding;
	}

	// Generates coefficients for basis accelerations 
	
	function Get_Coordinates() {
		for(local i=0; i<2; i++) {
			::portal[i] = portal_temp[i]
			::origin[i] = ::portal[i].GetOrigin();
			::local_matrix[i] = Vectors_To_Matrix(::portal[i].GetUpVector(), ::portal[i].GetLeftVector(), ::portal[i].GetForwardVector());
			::local_inv[i] = Vectors_To_Matrix_Inv(::portal[i].GetUpVector(), ::portal[i].GetLeftVector(), ::portal[i].GetForwardVector());
			::local_g[i] = Matrix_Vector_Mult_3D(local_inv[i], Vector(0.0,0.0,g));
		}
	}
	
	function Generate() {

		matrix <- [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]; 

		rside <- [0.0, 0.0, 0.0, 0.0]
		rside[0] = g * (origin[1] - origin[0]).z
		rside[1] =  local_g[1].x - local_g[0].x
		rside[2] = -local_g[1].y - local_g[0].y
		rside[3] = -local_g[1].z - local_g[0].z
		
		sign <- [-1, -1, 1, 1]
		loc_coords <- [[null, null], [null, null]];
		el_coords <- [[null, null], [null, null]];
		aux_vectors <- [[null, null], [null, null]];
		potential <- [[null, null], [null, null]];  // Potentials and their derivatives
		for(local i=0; i<2; i++) {
			for(local j=0; j<2; j++) {
				if(i != j) {
					loc_coords[i][j] = Matrix_Vector_Mult_3D(local_inv[j], origin[i] - origin[j]);
					el_coords[i][j] = Get_Ellipsoidal_Coords(loc_coords[i][j]);
					aux_vectors[i][j] = Get_Aux_Vector(loc_coords[i][j], el_coords[i][j]);
					potential[i][j] = [Get_pot_Arr(loc_coords[i][j], el_coords[i][j], precision)]
					potential[i][j].append([0.0, 0.0, 0.0, 0.0])
					potential[i][j].append([0.0, 0.0, 0.0, 0.0])
					potential[i][j].append([0.0, 0.0, 0.0, 0.0])
					for (local j1=0; j1<4; j1++) {
						g_vec <- Get_g(loc_coords[i][j], el_coords[i][j], aux_vectors[i][j], j1, precision);
						g_vec = Matrix_Vector_Mult_3D(local_matrix[j], g_vec);
						g_vec = Matrix_Vector_Mult_3D(local_inv[i], g_vec);
						g_vec = Vector_To_Array(g_vec);
						for (local i1=1; i1<4; i1++)
							potential[i][j][i1][j1] = g_vec[i1-1]
					}
				} else {
					potential[i][j] = ::self_potential
					// In total, there are 7*2 = 14 integrations
				}
			} 
		}
		
		for (local i=0; i<4; i++)
			for(local j=0; j<4; j++)
				matrix[i][j] = potential[0][0][i][j] + sign[j] * potential[0][1][i][j] + sign[i] * potential[1][0][i][j] + sign[i] * sign[j] * potential[1][1][i][j];

		solution <- Solve_Linear_System(matrix, rside, 4);
		if (!solution) {
			printl("Error in solving linear equation")
			printl("Portal coordinates & orientations:")
			printl("Portal 0: " + ::origin[0] + " " + ::portal[0].GetAngles())
			printl("Portal 1: " + ::origin[1] + " " + ::portal[1].GetAngles())
			printl("")
			::constants = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
			return;
		}
		
		for (local i=0; i<4; i++) {
			::constants[0][i] = solution[i];
			::constants[1][i] = sign[i]*solution[i];
		}
		return;
	}

	// Remove deleted props from the array of props
	
	function Check_Remove() {
		for(local i=props.len(); i>0; i--) {
			if(!props[i-1] || (!props[i-1].IsValid() || (props[i-1].entindex() == 0))) {
				pushes[i-1].__KeyValueFromFloat("magnitude", 0.0);
				props.remove(i-1);
			}
		}
		return;
	}
	
	// Remove an invalid/null portal from the list
	
	function Portal_Remove() {
		for(local i=all_portals.len(); i>0; i--) {
			if(all_portals[i-1] && !(all_portals[i-1].IsValid())) {
				all_portals[i-1] = null;
				EntFireByHandle(detectors[i-1], "Disable", "", 0, null, null)
			}
		}
		return;
	}
	
	// Add new props to the array of props
	// Only checks props that are closer than the furthest prop in the array
	// If array is not yet filled, checks all props
	
	function Check_Add() {

		for(local i = 0; i < names.len(); i++){
			
			foreach(orig in origin) {
				if (props.len() < max_props) {
					curr <- Entities.FindByClassname(null, names[i])
				} else {
					prop_origin <- props.top().GetOrigin();
					dist <- min((prop_origin-origin[0]).Length(), (prop_origin-origin[1]).Length());

					curr <- Entities.FindByClassnameWithin(null, names[i], orig, dist);
				}
				while(curr) {
					
					for(count <- 0; count<props.len(); count++) {
						if(props[count] == curr)
							break;
					}
					
					if (count ==  props.len())
					{
						if (props.len() < max_props)
							props.append(curr);
						else
							props[max_props-1] = curr;
						
						if(props.len() > 1) {
							prev_prop_origin <- props[props.len()-2].GetOrigin();
							prev_dist <- min((prev_prop_origin-origin[0]).Length(), (prev_prop_origin-origin[1]).Length());
							
							prop_origin <- props.top().GetOrigin();
							dist <- min((prop_origin-origin[0]).Length(), (prop_origin-origin[1]).Length());
							
							index <- props.len()-1
							while(prev_dist > dist) {
								temp <- props[index-1];
								props[index-1] = props[index];
								props[index] = temp;
								index--;
								
								if (index == 0) break;
								prev_prop_origin <- props[index-1].GetOrigin();
								prev_dist <- min((prev_prop_origin-origin[0]).Length(), (prev_prop_origin-origin[1]).Length());
							}
						}
					}
					
					if (props.len() < max_props) {
						curr <- Entities.FindByClassname(curr, names[i])
					} else {
						if (count ==  max_props){
							prop_origin <- props.top().GetOrigin();
							dist <- min((prop_origin-origin[0]).Length(), (prop_origin-origin[1]).Length());
						}
						
						curr <- Entities.FindByClassnameWithin(curr, names[i], orig, dist);
					}
				}
				if (props.len() < max_props) break;
			}
		}
		return;
	}
	
	// Add a portal to the list and create an 'func_portal_detector object' to track it
	
	function Portal_Add() {
		
		ptl <- null;
		while(ptl = Entities.FindByClassname(ptl, "prop_portal")){
			
			for(count <- 0; count<all_portals.len(); count++) {
				if(all_portals[count] == ptl)
					break;
			}

			if (count ==  all_portals.len())
			{
				all_portals.append(ptl)
				detectors.append(Entities.CreateByClassname("func_portal_detector"));
				detectors[count].SetSize(Vector(-1,-1,-1), Vector(1, 1, 1));
				detectors[count].__KeyValueFromInt("LinkageGroupID", 0)
				
				model <- ptl.GetModelName();
				if(model == "models/portals/portal1.mdl")
					EntFireByHandle(detectors[count], "AddOutput", "OnStartTouchLinkedPortal !self:RunScriptCode:detected_blue = self.GetOrigin()", 0, detectors[count], detectors[count]);
				else
					EntFireByHandle(detectors[count], "AddOutput", "OnStartTouchLinkedPortal !self:RunScriptCode:detected_orange = self.GetOrigin()", 0, detectors[count], detectors[count]);
			}
			
			detectors[count].SetOrigin(ptl.GetOrigin())
			
			EntFireByHandle(detectors[count], "Toggle", "", 0, null, null);
			EntFireByHandle(detectors[count], "Toggle", "", 0, null, null);
		}
	}
	
	// Drawing the field
	// Portal physics is disabled while the field is being drawn
	
	if(redraw) {
		show_mins_glob
		Draw_Field(show_mins_glob, show_maxs_glob, calc_mins_glob, calc_maxs_glob, divx_glob, divy_glob, side_glob, err_term_glob);
		redraw = 0;
		pos = [0, 0, 0, null];
		return;
	}
	
	// Commands I used to generate an icon
	// I'll just put them here
	// portal_place 0 0 576 64 0.03125 -90 90 0
	// portal_place 0 1 576 64 127.96875 90 -90 0
	// script Draw_Field_At(Vector(480, 0, 0), Vector(672, 128, 128), Vector(448, 0, 0), Vector(704, 128, 1152), 12, 6, 1, 0.000004)

	// Set up portal detection
	
	Portal_Remove();
	Portal_Add();
	
	// Check portals
	
	portal_temp = [null, null];
		
	if(detected_blue && detected_orange) {
		portal_temp[0] = Entities.FindByClassnameNearest("prop_portal", detected_blue, 1.0);
		portal_temp[1] = Entities.FindByClassnameNearest("prop_portal", detected_orange, 1.0);
		::detected_blue = null;
		::detected_orange = null;
	}
	
	// No portals found
	
	if(!portal_temp[0] || !portal_temp[1]) {
		::portal[0] = null;
		::portal[1] = null;
		
		return;
	}
	
	// Generate coefficients and run physics for newly created portals
	
	if(!::portal[0] || !::portal[1]) {
		Get_Coordinates();
		Generate();
		
		Check_Remove();
		Check_Add();
		Run_Physics();
		return;
	} 
	
	// Generate coefficients and run physics for changed portals
	
	if (!Vector_Eq(::origin[0], portal_temp[0].GetOrigin()) || !Vector_Eq(::portal[0].GetAngles(), portal_temp[0].GetAngles()) || !Vector_Eq(::origin[1], portal_temp[1].GetOrigin()) || !Vector_Eq(::portal[1].GetAngles(), portal_temp[1].GetAngles())) {
		Get_Coordinates();
		Generate();
		
		Check_Remove();
		Check_Add();
		Run_Physics();
		return;
	}
	
	// Simply run physics if portals are unchanged
	
	Check_Remove();
	Check_Add();
	Run_Physics();
	return;
	
}

// Command to draw the field
// It is called on every tick

::Draw_Field <- function(show_mins, show_maxs, calc_mins, calc_maxs, divx, divy, side, err_term) {
	h <- Vector((calc_maxs.x - calc_mins.x) / divx, (calc_maxs.y - calc_mins.y) / divy, 0)
	tp <- 0;
	
	for (local i=pos[0]; i<divx; i++) {
		for (local j=pos[1]; j<divy; j++) {
			pad <- 1.0/16.0
			
			if (!::pos[3])
				::pos[3] = ((side > 0) ? (calc_maxs - Vector((i+0.5)*h.x, (j+0.5)*h.y, 0.0)) : (calc_mins + Vector((i+0.5)*h.x, (j+0.5)*h.y, 0.0))) 
			
			if (::stepsize == 0.0)
				::stepsize = 1.0
			
			while (::inbounds) {
				
				coeff <- [Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0)]
				lens <- [0, 0]
				
				for (local k=0; k<6; k++) {
				
					acceleration <- Vector(0.0, 0.0, 0.0);
			
					for(local j1=0; j1<2; j1++) {
						
						if(!portal[j1]) continue;
						
						local_v <- Matrix_Vector_Mult_3D(local_inv[j1], ::pos[3] + (coeff[0]*::RKF45[k][0] + coeff[1]*::RKF45[k][1] + coeff[2]*::RKF45[k][2] + coeff[3]*::RKF45[k][3] + coeff[4]*::RKF45[k][4])*::stepsize - origin[j1]);
						ellipt_v <- Get_Ellipsoidal_Coords(local_v);
						aux_v <- Get_Aux_Vector(local_v, ellipt_v);
						
						if (k==0 && fabs(local_v.z) < pad && ellipt_v.x < 5.5*sqrt(pad)) {
							tp <- j1 + 1
						}
						
						if(k==0)
							lens[j1] = ellipt_v.x;
						
						if (aux_v.LengthSqr() == 0.0) continue;
						
						g0 <- Get_g0(ellipt_v, aux_v) * constants[j1][0]
						g1x <- Get_g1x(local_v, ellipt_v, aux_v, precision) * constants[j1][1]
						g1y <- Get_g1y(local_v, ellipt_v, aux_v, precision) * constants[j1][2]
						g1z <- Get_g1z(local_v, ellipt_v, aux_v, precision) * constants[j1][3]
						
						acceleration += Matrix_Vector_Mult_3D(local_matrix[j1], (g0 + g1x + g1y + g1z)*-1);
					}
					
					if (line_sign == 0) 
						::line_sign = (side * (acceleration + Vector(0,0,-g)).z > 0) ? -1 : 1;
					coeff[k] = (acceleration + Vector(0,0,-g)) * line_sign
					
				}
				
				if (pos[2] == 0 && (coeff[0].z * line_sign) > 0) break;
				
				error <- (coeff[0]*::RKFerr[0] + coeff[1]*::RKFerr[1] + coeff[2]*::RKFerr[2] + coeff[3]*::RKFerr[3] + coeff[4]*::RKFerr[4] + coeff[5]*::RKFerr[5]).Length()*::stepsize
				
				next_stepsize <- 0.9 * ::stepsize * pow(err_term / error, 0.2)
				if (error > err_term) {
					::stepsize = next_stepsize
				} else {
					unculled <- ::pos[3]  + (coeff[0]*::RKF4[0] + coeff[1]*::RKF4[1] + coeff[2]*::RKF4[2] + coeff[3]*::RKF4[3] + coeff[4]*::RKF4[4])*::stepsize;
					next_point <- cull(::pos[3], unculled, show_mins, show_maxs);
					
					if (::inbounds == 1) {
						if (portal[0] && portal[1]) {
							temp <- lens[0]*lens[0] + lens[1]*lens[1]
							lens[0] *= (lens[0]/temp);
							lens[1] *= (lens[1]/temp);
							color <- gamma_corr(Orange, Blue, lens)
						} else
							color <- [255, 255, 0]
						DebugDrawLine(::pos[3], next_point, color[0], color[1], color[2], false, 1000)
					}
					
					if ((unculled.x <= show_maxs.x) && (unculled.x >= show_mins.x) && (unculled.y <= show_maxs.y) && (unculled.y >= show_mins.y) && (unculled.z <= show_maxs.z) && (unculled.z >= show_mins.z))
						::inbounds = 1;
					else
						if ((unculled.x <= calc_maxs.x) && (unculled.x >= calc_mins.x) && (unculled.y <= calc_maxs.y) && (unculled.y >= calc_mins.y) && (unculled.z <= calc_maxs.z) && (unculled.z >= calc_mins.z))
							::inbounds = 2;
						else
							::inbounds = 0;
					
					if (tp) {
						next_point <- ::pos[3] + (next_point - ::pos[3]) * 2.1*((origin[tp - 1] - ::pos[3]).Dot(portal[tp - 1].GetForwardVector()) / (next_point - ::pos[3]).Dot(portal[tp - 1].GetForwardVector()))
						::pos[3] = Matrix_Vector_Mult_3D(local_inv[tp - 1], next_point - origin[tp - 1]);
						::pos[3].y *= -1
						::pos[3].z *= -1
						::pos[3] = Matrix_Vector_Mult_3D(local_matrix[2 - tp], ::pos[3]) + origin[2 - tp];
						tp <- 0;
					}
					else {
						::pos[3] = next_point;
					}
					
					::pos[2]++;
					::stepsize = next_stepsize;
				}
			}
			::pos[3] = null;
			::stepsize = 0.0;
			::line_sign = 0;
			::inbounds = 2;
			::pos[2] = 0;
			::pos[1]++;
		}
		::pos[1] = 0;
		::pos[0]++;
	}
}

// Command to draw the field
// It is called once to start drawing

::Draw_Field_At <- function(show_mins, show_maxs, calc_mins, calc_maxs, divx, divy, side, err_term) {
	::show_mins_glob <- show_mins;
	::show_maxs_glob <- show_maxs;
	::calc_mins_glob <- calc_mins;
	::calc_maxs_glob <- calc_maxs;
	::divx_glob <- divx;
	::divy_glob <- divy;
	::side_glob <- side;
	::err_term_glob <- err_term;
	::redraw <- 1;
}

// Color averaging

::gamma_corr <- function(color1, color2, bias) {
	aver <- [0.0, 0.0, 0.0]
	for (local i=0; i<3; i++)
		aver[i] = abs(pow(pow(color1[i],gamma)*bias[0] + pow(color2[i],gamma)*bias[1], 1.0/gamma))
	return aver
}

// Culling

::cull <- function(point1, point2, mins, maxs) {
	point1_arr <- [point1.x, point1.y, point1.z]
	point2_arr <- [point2.x, point2.y, point2.z]
	planes <- [mins.x, mins.y, mins.z, maxs.x, maxs.y, maxs.z]
	axes <- [0,1,2,0,1,2]
	
	ans <- 1.0
	point <- point2
		
	for (local i=0; i<6; i++) {
		
		if ((point2_arr[axes[i]] - point1_arr[axes[i]]) == 0.0) continue;
		
		t <- (planes[i] - point1_arr[axes[i]]) / (point2_arr[axes[i]] - point1_arr[axes[i]])
		if ((t - -0.0) > 0.0 && t <= 1.0) {
			point_arr <- [point1_arr[0] + (point2_arr[0] - point1_arr[0]) * t, point1_arr[1] + (point2_arr[1] - point1_arr[1]) * t, point1_arr[2] + (point2_arr[2] - point1_arr[2]) * t]
			point_arr[axes[i]] = planes[i]
			if ((point_arr[0] <= maxs.x) && (point_arr[0] >= mins.x) && (point_arr[1] <= maxs.y) && (point_arr[1] >= mins.y) && (point_arr[2] <= maxs.z) && (point_arr[2] >= mins.z)) {
				if (t < ans) {
					ans = t;
					point <- Vector(point_arr[0], point_arr[1], point_arr[2])
				}
			}
		}
	}
	
	return point
}
	
::mod_logic <- function() {

	// Generate clock entity

	if(!Entities.FindByName(null,"loop_timer")){
		timer <- Entities.CreateByClassname("logic_timer");
		timer.__KeyValueFromString("targetname","loop_timer");
		timer.__KeyValueFromFloat("RefireTime",0);
		timer.ConnectOutput("OnTimer", "main_loop");
		EntFire("loop_timer", "Enable");
	}

	// Generate an array of pusher entities
	// They are used to apply acceleration to props

	if(!Entities.FindByName(null,"push_0")){
		for (local i=0; i<max_props; i++) {
			pushes.append(Entities.CreateByClassname("point_push"));
			pushes[i].__KeyValueFromString("targetname","push_"+i);
			pushes[i].__KeyValueFromFloat("radius", 1.0);
			pushes[i].__KeyValueFromInt("spawnflags",16+4+2);
			EntFire("push_"+i, "Enable");
		}
	}
}
