if(!("Entities" in this)) return;
IncludeScript("maths");

names <- ["prop_weighted_cube", "prop_monster_box", "prop_paint_bomb", "npc_portal_turret_floor", "npc_security_camera", "prop_exploding_futbol"];
pushes <- [];
props <- [];
prop_id <- 0
detected_cyan <- null;
detected_purple <- null;
detected_yellow <- null;
detected_red <- null;

portal <- [null, null, null, null];
portal_temp <- [null, null, null, null];
origin <- [null, null, null, null];
local_matrix <- [null, null, null, null];
local_inv <- [null, null, null, null];
local_g <- [null, null, null, null];
constants <- [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]];

// Physical properties
const dt = 0.0333333333333333333333333333333333333333333;
const g = 600.0;

// Number of props for which we calculate acceleration
const max_props = 6;

// Integrals' precision (amount of intervals the range of integration is divided into)
const precision = 16

// Portal potential and it's derivatives induced by itself
self_potential <- [[0.0, 0.0, 0.0, 0.0]]
self_potential.append([0.0, -Integral(0.0, PI/2, E_1x, precision), 0.0, 0.0])
self_potential.append([0.0, 0.0, -Integral(0.0, PI/2, E_1y, precision), 0.0])
self_potential.append([0.0, 0.0, 0.0, -Integral(0.0, PI/2, E_1z, precision)])
// Technically, there are self_potential[0][3] and self_potential[3][0] terms, but they will cancel themselves out

main_loop <- function() {
	
	// Calculates and applies accelerations

	function Run_Physics() {
		
			for (local i=-2; i<props.len(); i++) {
			
				if (i == -2)
					curr <- Entities.FindByClassname(null, "player");
				if (i == -1)
					curr <- Entities.FindByClassname(curr, "player");
				if (i > -1)
					curr <- Entities.FindByName(null, props[i]);
				if(!curr) continue;
				
				if(Check_Funnels(curr)) continue;
				
				acceleration <- Vector(0.0, 0.0, 0.0);
				
				for(local j=0; j<4; j++) {
				
					if (!portal[j]) continue;
					
					local_v <- Matrix_Vector_Mult_3D(local_inv[j], curr.GetCenter() - origin[j]);
					ellipt_v <- Get_Ellipsoidal_Coords(local_v);
					aux_v <- Get_Aux_Vector(local_v, ellipt_v);
					
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
				pushes[i].__KeyValueFromFloat("magnitude", dir.Norm() / 33.333333333333333); // Now even 50 doesn't work! 1/3 of a hundred looks like it's working, but I'm not sure
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
	
	function Get_Coordinates_Cyan_Purple() {
		for(local i=0; i<2; i++) {
			::portal[i] = portal_temp[i]
			::origin[i] = ::portal[i].GetOrigin();
			::local_matrix[i] = Vectors_To_Matrix(::portal[i].GetUpVector(), ::portal[i].GetLeftVector(), ::portal[i].GetForwardVector());
			::local_inv[i] = Vectors_To_Matrix_Inv(::portal[i].GetUpVector(), ::portal[i].GetLeftVector(), ::portal[i].GetForwardVector());
			::local_g[i] = Matrix_Vector_Mult_3D(local_inv[i], Vector(0.0,0.0,g));
		}
	}
	
	function Get_Coordinates_Yellow_Red() {
		for(local i=2; i<4; i++) {
			::portal[i] = portal_temp[i]
			::origin[i] = ::portal[i].GetOrigin();
			::local_matrix[i] = Vectors_To_Matrix(::portal[i].GetUpVector(), ::portal[i].GetLeftVector(), ::portal[i].GetForwardVector());
			::local_inv[i] = Vectors_To_Matrix_Inv(::portal[i].GetUpVector(), ::portal[i].GetLeftVector(), ::portal[i].GetForwardVector());
			::local_g[i] = Matrix_Vector_Mult_3D(local_inv[i], Vector(0.0,0.0,g));
		}
	}


	// Generates coefficients for basis accelerations 
	
	function Generate_Cyan_Purple() {
		
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
			::constants = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
			return;
		}
		
		for (local i=0; i<4; i++) {
			::constants[0][i] = solution[i];
			::constants[1][i] = sign[i]*solution[i];
		}
		return;
	}
	
	function Generate_Yellow_Red() {
		
		matrix <- [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]; 

		rside <- [0.0, 0.0, 0.0, 0.0]
		rside[0] = g * (origin[3] - origin[2]).z
		rside[1] =  local_g[3].x - local_g[2].x
		rside[2] = -local_g[3].y - local_g[2].y
		rside[3] = -local_g[3].z - local_g[2].z
		
		sign <- [-1, -1, 1, 1]
		loc_coords <- [[null, null], [null, null]];
		el_coords <- [[null, null], [null, null]];
		aux_vectors <- [[null, null], [null, null]];
		potential <- [[null, null], [null, null]];  // Potentials and their derivatives
		for(local i=0; i<2; i++) {
			for(local j=0; j<2; j++) {
				if(i != j) {
					loc_coords[i][j] = Matrix_Vector_Mult_3D(local_inv[j+2], origin[i+2] - origin[j+2]);
					el_coords[i][j] = Get_Ellipsoidal_Coords(loc_coords[i][j]);
					aux_vectors[i][j] = Get_Aux_Vector(loc_coords[i][j], el_coords[i][j]);
					potential[i][j] = [Get_pot_Arr(loc_coords[i][j], el_coords[i][j], precision)]
					potential[i][j].append([0.0, 0.0, 0.0, 0.0])
					potential[i][j].append([0.0, 0.0, 0.0, 0.0])
					potential[i][j].append([0.0, 0.0, 0.0, 0.0])
					for (local j1=0; j1<4; j1++) {
						g_vec <- Get_g(loc_coords[i][j], el_coords[i][j], aux_vectors[i][j], j1, precision);
						g_vec = Matrix_Vector_Mult_3D(local_matrix[j+2], g_vec);
						g_vec = Matrix_Vector_Mult_3D(local_inv[i+2], g_vec);
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
			printl("Portal 2: " + ::origin[2] + " " + ::portal[2].GetAngles())
			printl("Portal 3: " + ::origin[3] + " " + ::portal[3].GetAngles())
			printl("")
			::constants = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
			return;
		}
		
		for (local i=0; i<4; i++) {
			::constants[2][i] = solution[i];
			::constants[3][i] = sign[i]*solution[i];
		}
		return;
	}
	
	function Generate_Both() {

		matrix <-     [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]];   //8x8 matrix. Yes. Because we need 8 coefficients
		matrix.extend([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]);
		matrix.extend([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]);
		matrix.extend([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]);
		
		rside <- [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		rside[0] = g * (origin[1] - origin[0]).z
		rside[1] =  local_g[1].x - local_g[0].x
		rside[2] = -local_g[1].y - local_g[0].y
		rside[3] = -local_g[1].z - local_g[0].z
		rside[4] = g * (origin[3] - origin[2]).z
		rside[5] =  local_g[3].x - local_g[2].x
		rside[6] = -local_g[3].y - local_g[2].y
		rside[7] = -local_g[3].z - local_g[2].z
		
		sign <- [-1, -1, 1, 1]
		loc_coords <- [[null, null, null, null], [null, null, null, null], [null, null, null, null], [null, null, null, null]];
		el_coords <- [[null, null, null, null], [null, null, null, null], [null, null, null, null], [null, null, null, null]];
		aux_vectors <- [[null, null, null, null], [null, null, null, null], [null, null, null, null], [null, null, null, null]];
		potential <- [[null, null, null, null], [null, null, null, null], [null, null, null, null], [null, null, null, null]];  // Potentials and their derivatives
		for(local i=0; i<4; i++) {
			for(local j=0; j<4; j++) {
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
					// In total, there are 7*(16-4) = 84 integrations
				}
			} 
		}
		
		for (local i=0; i<4; i++) {
			for(local j=0; j<4; j++) { 	
				matrix[i][j]     = potential[0][0][i][j] + sign[j] * potential[0][1][i][j] + sign[i] * potential[1][0][i][j] + sign[i] * sign[j] * potential[1][1][i][j];
				matrix[i][4+j]   = potential[0][2][i][j] + sign[j] * potential[0][3][i][j] + sign[i] * potential[1][2][i][j] + sign[i] * sign[j] * potential[1][3][i][j];
				matrix[4+i][j]   = potential[2][0][i][j] + sign[j] * potential[2][1][i][j] + sign[i] * potential[3][0][i][j] + sign[i] * sign[j] * potential[3][1][i][j];
				matrix[4+i][4+j] = potential[2][2][i][j] + sign[j] * potential[2][3][i][j] + sign[i] * potential[3][2][i][j] + sign[i] * sign[j] * potential[3][3][i][j];
			}
		}
		
		solution <- Solve_Linear_System(matrix, rside, 8);
		if (!solution) {
			printl("Error in solving linear equation")
			printl("Portal coordinates & orientations:")
			printl("Portal 0: " + ::origin[0] + " " + ::portal[0].GetAngles())
			printl("Portal 1: " + ::origin[1] + " " + ::portal[1].GetAngles())
			printl("Portal 2: " + ::origin[2] + " " + ::portal[2].GetAngles())
			printl("Portal 3: " + ::origin[3] + " " + ::portal[3].GetAngles())
			printl("")
			::constants = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
			return;
		}
		
		for (local i=0; i<4; i++) {
			::constants[0][i] = solution[i];
			::constants[1][i] = sign[i]*solution[i];
			::constants[2][i] = solution[i+4];
			::constants[3][i] = sign[i]*solution[i+4];
		}
		
		return;
	}

	// Remove deleted props from the array of props
	
	function Check_Remove() {
		for(local i=props.len(); i>0; i--) {
			if(!Entities.FindByName(null, props[i-1])) {
				pushes[i-1].__KeyValueFromFloat("magnitude", 0.0);
				props.remove(i-1);
			}
		}
		return;
	}
	
	// Add new props to the array of props
	// Only checks props that are closer than the furthest prop in the array
	// If array is not yet filled, checks all props
	
	function Check_Add() {
		
		for(local i = 0; i < names.len(); i++){
			
			for(local j = 0; j < 4; j++) {
				
				if (!::portal[j]) continue;
				
				if (props.len() < max_props) {
					curr <- Entities.FindByClassname(null, names[i])
				} else {
					prop_origin <- Entities.FindByName(null, props.top()).GetOrigin();
					dist <- Portal_Distance(prop_origin);

					curr <- Entities.FindByClassnameWithin(null, names[i], ::origin[j], dist);
				}
				while(curr) {
				
					if(curr.GetName() == "") curr.__KeyValueFromString("targetname","pushable_"+(prop_id++));
					
					for(count <- 0; count<props.len(); count++) {
						if(props[count] == curr.GetName())
							break;
					}
					
					if (count ==  props.len())
					{
						if (props.len() < max_props)
							props.append(curr.GetName());
						else
							props[max_props-1] = curr.GetName();
						
						if(props.len() > 1) {
							prev_prop_origin <- Entities.FindByName(null, props[props.len()-2]).GetOrigin();
							prev_dist <- Portal_Distance(prev_prop_origin);
							
							prop_origin <- Entities.FindByName(null, props.top()).GetOrigin();
							dist <- Portal_Distance(prop_origin);
							
							index <- props.len()-1
							while(prev_dist > dist) {
								temp <- props[index-1];
								props[index-1] = props[index];
								props[index] = temp;
								index--;
								
								if (index == 0) break;
								prev_prop_origin <- Entities.FindByName(null, props[index-1]).GetOrigin();
								prev_dist <- Portal_Distance(prev_prop_origin);
							}
						}
					}
					
					if (props.len() < max_props) {
						curr <- Entities.FindByClassname(curr, names[i])
					} else {
						if (count ==  max_props){
							prop_origin <- Entities.FindByName(null, props.top()).GetOrigin();
							dist <- Portal_Distance(prop_origin);
						}
						
						curr <- Entities.FindByClassnameWithin(curr, names[i], ::origin[j], dist);
					}
				}
				if (props.len() < max_props) break;
			}
		}
		return;
	}
	
	function Portal_Distance(other) {
		if (::portal[0] && ::portal[2]) {
			dist1 <- min((other-origin[0]).Length(), (other-origin[1]).Length());
			dist2 <- min((other-origin[2]).Length(), (other-origin[3]).Length());
			return min(dist1, dist2);
		}
		if (::portal[0])
			return min((other-origin[0]).Length(), (other-origin[1]).Length());
		if (::portal[2])
			return min((other-origin[2]).Length(), (other-origin[3]).Length());
		return 9999999
	}
	
	detector <- null
	while(detector = Entities.FindByName(detector,"detector_cyan"))
		detector.Destroy();
	while(detector = Entities.FindByName(detector,"detector_yellow"))
		detector.Destroy();
	while(detector = Entities.FindByName(detector,"detector_purple"))
		detector.Destroy();
	while(detector = Entities.FindByName(detector,"detector_red")) 
		detector.Destroy();
	
	ptl <- null;
	while(ptl = Entities.FindByClassname(ptl, "prop_portal")){
		model <- ptl.GetModelName();
		if(model == "models/portals/portal1.mdl"){
			if (ptl.GetTeam() == 3) {
				detector = Entities.CreateByClassname("func_portal_detector");
				detector.__KeyValueFromString("targetname", "detector_cyan");
				detector.__KeyValueFromInt("LinkageGroupID", 1)
				detector.SetOrigin(ptl.GetOrigin())
				detector.SetSize(Vector(-1,-1,-1), Vector(1, 1, 1))
			}
			if (ptl.GetTeam() == 2) {
				detector = Entities.CreateByClassname("func_portal_detector");
				detector.__KeyValueFromString("targetname", "detector_yellow");
				detector.__KeyValueFromInt("LinkageGroupID", 2)
				detector.SetOrigin(ptl.GetOrigin())
				detector.SetSize(Vector(-1,-1,-1), Vector(1, 1, 1))
			}
		} else {
			if (ptl.GetTeam() == 3) {
				detector = Entities.CreateByClassname("func_portal_detector");
				detector.__KeyValueFromString("targetname", "detector_purple");
				detector.__KeyValueFromInt("LinkageGroupID", 1)
				detector.SetOrigin(ptl.GetOrigin())
				detector.SetSize(Vector(-1,-1,-1), Vector(1, 1, 1))
			}
			if (ptl.GetTeam() == 2) {
				detector = Entities.CreateByClassname("func_portal_detector");
				detector.__KeyValueFromString("targetname", "detector_red");
				detector.__KeyValueFromInt("LinkageGroupID", 2)
				detector.SetOrigin(ptl.GetOrigin())
				detector.SetSize(Vector(-1,-1,-1), Vector(1, 1, 1))
			}
		}
	}
	
	EntFire("detector_cyan", "AddOutput", "OnStartTouchLinkedPortal !self:RunScriptCode:detected_cyan = self.GetOrigin()");
	EntFire("detector_purple", "AddOutput", "OnStartTouchLinkedPortal !self:RunScriptCode:detected_purple = self.GetOrigin()");
	EntFire("detector_yellow", "AddOutput", "OnStartTouchLinkedPortal !self:RunScriptCode:detected_yellow = self.GetOrigin()");
	EntFire("detector_red", "AddOutput", "OnStartTouchLinkedPortal !self:RunScriptCode:detected_red = self.GetOrigin()");
	
	EntFire("detector_cyan", "Enable");
	EntFire("detector_purple", "Enable");
	EntFire("detector_yellow", "Enable");
	EntFire("detector_red", "Enable");

	// Check portals
	
	portal_temp = [null, null, null, null];
		
	if(detected_cyan && detected_purple) {
		portal_temp[0] = Entities.FindByClassnameNearest("prop_portal", detected_cyan, 1.0);
		portal_temp[1] = Entities.FindByClassnameNearest("prop_portal", detected_purple, 1.0);
		::detected_cyan = null;
		::detected_purple = null;
	}
	
	
	if(detected_yellow && detected_red) {
		portal_temp[2] = Entities.FindByClassnameNearest("prop_portal", detected_yellow, 1.0);
		portal_temp[3] = Entities.FindByClassnameNearest("prop_portal", detected_red, 1.0);
		::detected_yellow = null;
		::detected_red = null;
	}
	
	// No portals found
	if((!portal_temp[0] || !portal_temp[1]) && (!portal_temp[2] || !portal_temp[3])) {
		::portal[0] = null;
		::portal[1] = null;
		::portal[2] = null;
		::portal[3] = null;
		
		return;
	}

	changed <- false
	
	if(portal_temp[0] && portal_temp[1]) {
		if(!::portal[0] || !::portal[1]) {
			changed = true;
			Get_Coordinates_Cyan_Purple();
		} else {
			if (!Vector_Eq(::origin[0], portal_temp[0].GetOrigin()) || !Vector_Eq(::portal[0].GetAngles(), portal_temp[0].GetAngles()) || !Vector_Eq(::origin[1], portal_temp[1].GetOrigin()) || !Vector_Eq(::portal[1].GetAngles(), portal_temp[1].GetAngles())) {
				changed = true;
				Get_Coordinates_Cyan_Purple();
			}
		}
	} else {
		if(::portal[0] && ::portal[1])
			changed = true;
	}
	
	if(portal_temp[2] && portal_temp[3]) {
		if(!::portal[2] || !::portal[3]) {
			changed = true;
			Get_Coordinates_Yellow_Red();
		} else {
			if (!Vector_Eq(::origin[2], portal_temp[2].GetOrigin()) || !Vector_Eq(::portal[2].GetAngles(), portal_temp[2].GetAngles()) || !Vector_Eq(::origin[3], portal_temp[3].GetOrigin()) || !Vector_Eq(::portal[3].GetAngles(), portal_temp[3].GetAngles())) {
				changed = true;
				Get_Coordinates_Yellow_Red();
			}
		}
	} else {
		if(::portal[2] && ::portal[3])
			changed = true;
	}
	
	// Simply run physics if portals are unchanged
	
	if (!changed) {
		Check_Remove();
		Check_Add();
		Run_Physics();
		return;
	}
	
	// Generate coefficients and run physics
	
	if (portal_temp[0] && portal_temp[1] && portal_temp[2] && portal_temp[3]) {
		Generate_Both();
		
		Check_Remove();
		Check_Add();
		Run_Physics();
		return;
	}
	
	if (portal_temp[0] && portal_temp[1]) {
		::portal[2] = null;
		::portal[3] = null;
		Generate_Cyan_Purple();
		
		Check_Remove();
		Check_Add();
		Run_Physics();
		return;
	}
	
	if (portal_temp[2] && portal_temp[3]) {
		::portal[0] = null;
		::portal[1] = null;
		Generate_Yellow_Red();
		
		Check_Remove();
		Check_Add();
		Run_Physics();
		return;
	}
	
	return;
}

::mod_logic <- function() {

	// Generate clock entity

	if(!Entities.FindByName(null,"loop_timer")){
		timer <- Entities.CreateByClassname("logic_timer");
		timer.__KeyValueFromString("targetname","loop_timer");
		timer.__KeyValueFromFloat("RefireTime",dt);
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
