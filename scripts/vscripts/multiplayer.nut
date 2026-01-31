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
g0_constant <- [null, null, null, null];

// Physical properties
const dt = 0.0333333333333333333333333333333333333333333;
const g = 600.0;
const b_ = 32.0;
const a_ = 56.0;
const a2tb2 = 3211264.0;

// Number of props for which we calculate acceleration
const max_props = 6;

// Precalculated constants
const d1x = 151484.09686731211638346807701780282673155084349221841210723666217;
const d1y = 65259.463559493120080357854500977681313126585891064972886646114392;
const d1z = 45610.448033096360092265308747487004729466142379513813304004751523;

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
					aux_v <- Get_Aux_Vec(local_v, ellipt_v);
					g0_acc <- Get_g0_Vec(local_v, ellipt_v, aux_v) * g0_constant[j]
					
					p1 <- j;
					p2 <- (j > 1) ? 5-j : 1-j;
					g1x <- Get_g1x_Vec(local_v, ellipt_v, aux_v) * (local_g[p1].x - local_g[p2].x)
					g1y <- Get_g1y_Vec(local_v, ellipt_v, aux_v) * (local_g[p1].y + local_g[p2].y)
					g1z <- Get_g1z_Vec(local_v, ellipt_v, aux_v) * (local_g[p1].z + local_g[p2].z)
					
					acceleration += Matrix_Vector_Mult_3D(local_matrix[j], g1x + g1y + g1z - g0_acc); // minus sign for g1 is already accounted in the gravity vector
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
		
	// Funnels should nullify redirected gravity too
	
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
			::local_g[i] = Matrix_Vector_Mult_3D(local_inv[i], Vector(0.0,0.0,-g));
		}
	}
	
	function Get_Coordinates_Yellow_Red() {
		for(local i=2; i<4; i++) {
			::portal[i] = portal_temp[i]
			::origin[i] = ::portal[i].GetOrigin();
			::local_matrix[i] = Vectors_To_Matrix(::portal[i].GetUpVector(), ::portal[i].GetLeftVector(), ::portal[i].GetForwardVector());
			::local_inv[i] = Vectors_To_Matrix_Inv(::portal[i].GetUpVector(), ::portal[i].GetLeftVector(), ::portal[i].GetForwardVector());
			::local_g[i] = Matrix_Vector_Mult_3D(local_inv[i], Vector(0.0,0.0,-g));
		}
	}


	// Generates coefficients for basis accelerations 
	
	function Generate_Cyan_Purple() {
		p1 <- 0;
		p2 <- 1;
		
		local_v_12 <- Matrix_Vector_Mult_3D(local_inv[p2], origin[p1] - origin[p2]);
		ellipt_v_12 <- Get_Ellipsoidal_Coords(local_v_12);
		local_v_21 <- Matrix_Vector_Mult_3D(local_inv[p1], origin[p2] - origin[p1]);
		ellipt_v_21 <- Get_Ellipsoidal_Coords(local_v_21);
		
		if (local_v_12.x != 0.0) {
			aux_12 <- local_v_12.z > 0 ? (ellipt_v_12.y*ellipt_v_12.z / a2tb2) : (-ellipt_v_12.y*ellipt_v_12.z / a2tb2)
			vx_12 <- -(local_g[p1].x - local_g[p2].x)*0.5 * d1x * (-local_v_12.x * Integral(atan(ellipt_v_12.x/b_), PI/2, E_1x, 128))
			vy_12 <- -(local_g[p1].y + local_g[p2].y)*0.5 * d1y * (-local_v_12.y * Integral(atan(ellipt_v_12.x/b_), PI/2, E_1y, 128))
			vz_12 <- -(local_g[p1].z + local_g[p2].z)*0.5 * d1z * (aux_12 - local_v_12.z * Integral(atan(ellipt_v_12.x/b_), PI/2, E_1z, 128))
			
			aux_21 <- local_v_21.z > 0 ? (ellipt_v_21.y*ellipt_v_21.z / a2tb2) : (-ellipt_v_21.y*ellipt_v_21.z / a2tb2)
			vx_21 <- -(local_g[p2].x - local_g[p1].x)*0.5 * d1x * (-local_v_21.x * Integral(atan(ellipt_v_21.x/b_), PI/2, E_1x, 128))
			vy_21 <- -(local_g[p2].y + local_g[p1].y)*0.5 * d1y * (-local_v_21.y * Integral(atan(ellipt_v_21.x/b_), PI/2, E_1y, 128))
			vz_21 <- -(local_g[p2].z + local_g[p1].z)*0.5 * d1z * (aux_21 - local_v_21.z * Integral(atan(ellipt_v_21.x/b_), PI/2, E_1z, 128))
			
			::g0_constant[p1] = (g*(origin[p1] - origin[p2]).z + vx_12 + vy_12 + vz_12 - vx_21 - vy_21 - vz_21) / (Integral(0.0, atan(ellipt_v_12.x/b_), E_0, 128) + Integral(0.0, atan(ellipt_v_21.x/b_), E_0, 128))
			::g0_constant[p2] = -g0_constant[p1];
		} else {
			::g0_constant[p1] = 0;
			::g0_constant[p2] = 0
		}
		return;
	}
	
	function Generate_Yellow_Red() {
		p1 <- 2;
		p2 <- 3;
		
		local_v_12 <- Matrix_Vector_Mult_3D(local_inv[p2], origin[p1] - origin[p2]);
		ellipt_v_12 <- Get_Ellipsoidal_Coords(local_v_12);
		local_v_21 <- Matrix_Vector_Mult_3D(local_inv[p1], origin[p2] - origin[p1]);
		ellipt_v_21 <- Get_Ellipsoidal_Coords(local_v_21);
		
		if (local_v_12.x != 0.0) {
			aux_12 <- local_v_12.z > 0 ? (ellipt_v_12.y*ellipt_v_12.z / a2tb2) : (-ellipt_v_12.y*ellipt_v_12.z / a2tb2)
			vx_12 <- -(local_g[p1].x - local_g[p2].x)*0.5 * d1x * (-local_v_12.x * Integral(atan(ellipt_v_12.x/b_), PI/2, E_1x, 128))
			vy_12 <- -(local_g[p1].y + local_g[p2].y)*0.5 * d1y * (-local_v_12.y * Integral(atan(ellipt_v_12.x/b_), PI/2, E_1y, 128))
			vz_12 <- -(local_g[p1].z + local_g[p2].z)*0.5 * d1z * (aux_12 - local_v_12.z * Integral(atan(ellipt_v_12.x/b_), PI/2, E_1z, 128))
			
			aux_21 <- local_v_21.z > 0 ? (ellipt_v_21.y*ellipt_v_21.z / a2tb2) : (-ellipt_v_21.y*ellipt_v_21.z / a2tb2)
			vx_21 <- -(local_g[p2].x - local_g[p1].x)*0.5 * d1x * (-local_v_21.x * Integral(atan(ellipt_v_21.x/b_), PI/2, E_1x, 128))
			vy_21 <- -(local_g[p2].y + local_g[p1].y)*0.5 * d1y * (-local_v_21.y * Integral(atan(ellipt_v_21.x/b_), PI/2, E_1y, 128))
			vz_21 <- -(local_g[p2].z + local_g[p1].z)*0.5 * d1z * (aux_21 - local_v_21.z * Integral(atan(ellipt_v_21.x/b_), PI/2, E_1z, 128))
			
			::g0_constant[p1] = (g*(origin[p1] - origin[p2]).z + vx_12 + vy_12 + vz_12 - vx_21 - vy_21 - vz_21) / (Integral(0.0, atan(ellipt_v_12.x/b_), E_0, 128) + Integral(0.0, atan(ellipt_v_21.x/b_), E_0, 128))
			::g0_constant[p2] = -g0_constant[p1];
		} else {
			::g0_constant[p1] = 0;
			::g0_constant[p2] = 0
		}
		return;
	}
	
	function Generate_Both() {
		
		potential_0 <- [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0,0.0]];
		potential_1 <- [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0,0.0]];
		
		//Calculating potentials at a portal I from a portal J
		for(local i=0; i<4; i++) {
			for(local j=0; j<4; j++) {
				if (i != j) {
					local_v <- Matrix_Vector_Mult_3D(local_inv[j], origin[i] - origin[j]);
					ellipt_v <- Get_Ellipsoidal_Coords(local_v);
					potential_0[i][j] = Integral(0.0, atan(ellipt_v.x/b_), E_0, 128);
					
					p1 <- j;
					p2 <- (j > 1) ? 5-j : 1-j;
					
					aux <- local_v.z > 0 ? (ellipt_v.y*ellipt_v.z / a2tb2) : (-ellipt_v.y*ellipt_v.z / a2tb2)
					vx <- -(local_g[p1].x - local_g[p2].x)*0.5 * d1x * (-local_v.x * Integral(atan(ellipt_v.x/b_), PI/2, E_1x, 128))
					vy <- -(local_g[p1].y + local_g[p2].y)*0.5 * d1y * (-local_v.y * Integral(atan(ellipt_v.x/b_), PI/2, E_1y, 128))
					vz <- -(local_g[p1].z + local_g[p2].z)*0.5 * d1z * (aux - local_v.z * Integral(atan(ellipt_v.x/b_), PI/2, E_1z, 128))

					potential_1[i][j] = vx + vy + vz;
				}
			}
		}
		
		matrix <- [[0.0, 0.0], [0.0, 0.0]];
		matrix[0][0] = potential_0[0][1] + potential_0[1][0];
		matrix[1][1] = potential_0[2][3] + potential_0[3][2];
		matrix[0][1] = -potential_0[0][2] + potential_0[0][3] + potential_0[1][2] - potential_0[1][3];
		matrix[1][0] = -potential_0[2][0] + potential_0[2][1] + potential_0[3][0] - potential_0[3][1];
		
		rside <- [0.0, 0.0]
		rside[0] = g * (origin[0] - origin[1]).z + potential_1[0][1] + potential_1[0][2] + potential_1[0][3] - potential_1[1][0] - potential_1[1][2] - potential_1[1][3];
		rside[1] = g * (origin[2] - origin[3]).z + potential_1[2][0] + potential_1[2][1] + potential_1[2][3] - potential_1[3][0] - potential_1[3][1] - potential_1[3][2];
		
		solution <- Solve_2D_System(matrix, rside);
		::g0_constant[0] = solution[0];
		::g0_constant[1] = -solution[0];
		::g0_constant[2] = solution[1];
		::g0_constant[3] = -solution[1];
		
		return;
	}
	
	function Calculate_Potential() {
		potential_0 <- [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0,0.0]];
		potential_1 <- [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0,0.0]];
		
		//Calculating potentials at a portal I from a portal J
		for(local i=0; i<4; i++) {
			for(local j=0; j<4; j++) {
				if (i != j) {
					local_v <- Matrix_Vector_Mult_3D(local_inv[j], origin[i] - origin[j]);
					ellipt_v <- Get_Ellipsoidal_Coords(local_v);
					potential_0[i][j] = Integral(0.0, atan(ellipt_v.x/b_), E_0, 128) * g0_constant[j];
					
					p1 <- j;
					p2 <- (j > 1) ? 5-j : 1-j;
					
					aux <- local_v.z > 0 ? (ellipt_v.y*ellipt_v.z / a2tb2) : (-ellipt_v.y*ellipt_v.z / a2tb2)
					vx <- -(local_g[p1].x - local_g[p2].x)*0.5 * d1x * (-local_v.x * Integral(atan(ellipt_v.x/b_), PI/2, E_1x, 128))
					vy <- -(local_g[p1].y + local_g[p2].y)*0.5 * d1y * (-local_v.y * Integral(atan(ellipt_v.x/b_), PI/2, E_1y, 128))
					vz <- -(local_g[p1].z + local_g[p2].z)*0.5 * d1z * (aux - local_v.z * Integral(atan(ellipt_v.x/b_), PI/2, E_1z, 128))

					potential_1[i][j] = vx + vy + vz;
				} else {
					p1 <- j;
					p2 <- (j > 1) ? 5-j : 1-j;
					
					potential_1[i][j] = (local_g[p1].z + local_g[p2].z)*0.5 * d1z / 1792.0;
				}
			}
		}
		for (local i=0; i<4; i++)
			printl("portal " + i + ": " + (g * portal[i].GetOrigin().z + potential_0[i][0] + potential_0[i][1] + potential_0[i][2] + potential_0[i][3] + potential_1[i][0] + potential_1[i][1] + potential_1[i][2] + potential_1[i][3]))
		printl("");
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
		max_dist <- -1.0;
		max_index <- -1;
		
		for(local i = 0; i < names.len(); i++){
			foreach(orig in origin) {
				if (props.len() < max_props) {
					curr <- Entities.FindByClassname(null, names[i])
				} else {
					max_dist = -1.0;
					max_index = -1;
					for(local i = 0; i<props.len(); i++) {
						prop_origin <- Entities.FindByName(null, props[i]).GetOrigin();
						dist1 <- min((prop_origin-origin[0]).Length(), (prop_origin-origin[1]).Length());
						dist2 <- min((prop_origin-origin[2]).Length(), (prop_origin-origin[3]).Length());
						dist <- min(dist1, dist2);
						if(dist > max_dist) {
							max_dist = dist;
							max_index = i;
						}
					}
					curr <- Entities.FindByClassnameWithin(null, names[i], orig, max_dist);
				}
				while(curr) {
				
					if(curr.GetName() == "") curr.__KeyValueFromString("targetname","pushable_"+(prop_id++));
					if(!curr.IsValid()) continue;
					
					for(count <- 0; count<props.len(); count++) {
						if(props[count] == curr.GetName())
							break;
					}
					
					if (count ==  props.len())
					{
						if (props.len() < max_props)
							props.append(curr.GetName());
						else
							props[max_index] = curr.GetName();
							
						if (props.len() == max_props) {
							max_dist = -1.0;
							max_index = -1;
							for(local i = 0; i<props.len(); i++) {
								prop_origin <- Entities.FindByName(null, props[i]).GetOrigin();
								dist1 <- min((prop_origin-origin[0]).Length(), (prop_origin-origin[1]).Length());
								dist2 <- min((prop_origin-origin[2]).Length(), (prop_origin-origin[3]).Length());
								dist <- min(dist1, dist2);
								if(dist > max_dist) {
									max_dist = dist;
									max_index = i;
								}
							}
						}
					}
					
					if (props.len() < max_props)
						curr <- Entities.FindByClassname(curr, names[i])
					else
						curr <- Entities.FindByClassnameWithin(curr, names[i], orig, max_dist);
				}
				if (props.len() < max_props) break;
			}
		}
		return;
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
		//Calculate_Potential();
		
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
