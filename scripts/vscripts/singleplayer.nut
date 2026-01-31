if(!("Entities" in this)) return;
IncludeScript("maths");

names <- ["prop_weighted_cube", "prop_monster_box", "prop_paint_bomb", "npc_portal_turret_floor", "npc_security_camera", "prop_exploding_futbol"];
pushes <- [];
props <- [];
prop_id <- 0
detected_blue <- null;
detected_orange <- null;

portal <- [null, null];
portal_temp <- [null, null];
origin <- [null, null];
local_matrix <- [null, null,];
local_inv <- [null, null];
local_g <- [null, null];
g0_constant <- [null, null];

// Physical properties
const dt = 0.0333333333333333333;
const g = 600.0;
const b_ = 32.0;
const a_ = 56.0;
const a2tb2 = 3211264.0;

// Number of props which we are tracking
const max_props = 4;

// Precalculated constants
const d1x = 151484.09686731211638346807701780282673155084349221841210723666217;
const d1y = 65259.463559493120080357854500977681313126585891064972886646114392;
const d1z = 45610.448033096360092265308747487004729466142379513813304004751523;

main_loop <- function() {
	
	// Calculates and applies accelerations

	Run_Physics <- function() {
		
			for (local i=-1; i<props.len(); i++) {
			
				if (i < 0)
					curr <- GetPlayer();
				else
					curr <- Entities.FindByName(null, props[i]);
				if(!curr) continue;
				
				if(Check_Funnels(curr)) continue;
				
				acceleration <- Vector(0.0, 0.0, 0.0);
				
				for(local j=0; j<2; j++) {
					
					local_v <- Matrix_Vector_Mult_3D(local_inv[j], curr.GetCenter() - origin[j]);
					ellipt_v <- Get_Ellipsoidal_Coords(local_v);
					aux_v <- Get_Aux_Vec(local_v, ellipt_v);
					g0_acc <- Get_g0_Vec(local_v, ellipt_v, aux_v) * g0_constant[j]
					
					p1 <- j;
					p2 <- 1-j;
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

	// Generates coefficients for basis accelerations 
	
	function Get_Coordinates() {
		for(local i=0; i<2; i++) {
			::portal[i] = portal_temp[i]
			::origin[i] = ::portal[i].GetOrigin();
			::local_matrix[i] = Vectors_To_Matrix(::portal[i].GetUpVector(), ::portal[i].GetLeftVector(), ::portal[i].GetForwardVector());
			::local_inv[i] = Vectors_To_Matrix_Inv(::portal[i].GetUpVector(), ::portal[i].GetLeftVector(), ::portal[i].GetForwardVector());
			::local_g[i] = Matrix_Vector_Mult_3D(local_inv[i], Vector(0.0,0.0,-g));
		}
	}
	
	function Generate() {
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
						dist <- min((prop_origin-origin[0]).Length(), (prop_origin-origin[1]).Length());
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
								dist <- min((prop_origin-origin[0]).Length(), (prop_origin-origin[1]).Length());
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

	// Find portals
	detector <- null;
	while(detector = Entities.FindByName(detector, "detector_blue"))
		detector.Destroy();
	while(detector = Entities.FindByName(detector, "detector_orange"))
		detector.Destroy();
	
	portal <- null;
	while(portal = Entities.FindByClassname(portal, "prop_portal")){
		model <- portal.GetModelName();
		if(model == "models/portals/portal1.mdl"){
			detector <- Entities.CreateByClassname("func_portal_detector");
			detector.__KeyValueFromString("targetname", "detector_blue");
			detector.__KeyValueFromInt("LinkageGroupID", 0)
			detector.SetOrigin(portal.GetOrigin())
			detector.SetSize(Vector(-1,-1,-1), Vector(1, 1, 1))
		} else {
			detector <- Entities.CreateByClassname("func_portal_detector");
			detector.__KeyValueFromString("targetname", "detector_orange");
			detector.__KeyValueFromInt("LinkageGroupID", 0)
			detector.SetOrigin(portal.GetOrigin())
			detector.SetSize(Vector(-1,-1,-1), Vector(1, 1, 1))
		}
	}
	
	EntFire("detector_blue", "AddOutput", "OnStartTouchLinkedPortal !self:RunScriptCode:detected_blue = self.GetOrigin()");
	EntFire("detector_orange", "AddOutput", "OnStartTouchLinkedPortal !self:RunScriptCode:detected_orange = self.GetOrigin()");
	EntFire("detector_blue", "Enable");
	EntFire("detector_orange", "Enable");
	
	// Check portals
	
	portal_temp = [null, null];
		
	if(detected_blue && detected_orange) {
		portal_temp[0] = Entities.FindByClassnameNearest("prop_portal", detected_blue, 1.0);
		portal_temp[1] = Entities.FindByClassnameNearest("prop_portal", detected_orange, 1.0);
		::detected_blue = null;
		::detected_orange = null;
	}
	
	if(!portal_temp[0] || !portal_temp[1]) {
		::portal[0] = null;
		::portal[1] = null;
		
		return;
	}
	
	
	if(!::portal[0] || !::portal[1]) {
		Get_Coordinates();
		Generate();
		
		Check_Remove();
		Check_Add();
		Run_Physics();
		return;
	} 
	
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
