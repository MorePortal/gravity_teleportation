if(!("Entities" in this)) return;
IncludeScript("maths");

names <- ["prop_weighted_cube", "prop_monster_box", "prop_paint_bomb", "npc_portal_turret_floor", "npc_security_camera", "prop_exploding_futbol"];
pushes <- [];
props <- [];
prop_id <- 0
detected_blue <- null;
detected_orange <- null;

const dt = 0.0333333333333333333;
const g = 600.0;
const b_ = 32.0;
const a_ = 56.0;
const max_props = 4;

main_loop <- function() {
	
	// Calculates and applies accelerations

	Run_Physics <- function() {
		
			for (local i=-1; i<props.len(); i++) {
				if(i == -1)
					curr <- GetPlayer();
				else
					curr <- Entities.FindByName(null, props[i]);
					
				if (Check_Funnels(curr)) continue;
				
				blue_local_v <- Matrix_Vector_Mult_3D(blue_local_inv, curr.GetCenter() - blue_origin)
				blue_ellipt_v <- Get_Ellipsoidal_Coords(blue_local_v);
				orange_local_v <- Matrix_Vector_Mult_3D(orange_local_inv, curr.GetCenter() - orange_origin)
				orange_ellipt_v <- Get_Ellipsoidal_Coords(orange_local_v);
				
				blue_aux_v <- Get_Aux_Vec(blue_local_v, blue_ellipt_v);
				blue_g0 <- Get_g0_Vec(blue_local_v, blue_ellipt_v, blue_aux_v) * dh
				blue_local_acc <- blue_g0
				
				orange_aux_v <- Get_Aux_Vec(orange_local_v, orange_ellipt_v);
				orange_g0 <- Get_g0_Vec(orange_local_v, orange_ellipt_v, orange_aux_v) * -dh
				orange_local_acc <- orange_g0
					
				blue_g1_blue <- Get_g1x_Vec(blue_local_v, blue_ellipt_v, blue_aux_v)*blue_g.x + Get_g1y_Vec(blue_local_v, blue_ellipt_v, blue_aux_v)*blue_g.y + Get_g1z_Vec(blue_local_v, blue_ellipt_v, blue_aux_v)*blue_g.z
				blue_g1_orange <- Get_g1x_Vec(blue_local_v, blue_ellipt_v, blue_aux_v)*-orange_g.x + Get_g1y_Vec(blue_local_v, blue_ellipt_v, blue_aux_v)*orange_g.y + Get_g1z_Vec(blue_local_v, blue_ellipt_v, blue_aux_v)*orange_g.z
				blue_local_acc <- blue_local_acc + (blue_g1_blue + blue_g1_orange)
				
				orange_g1_blue <- Get_g1x_Vec(orange_local_v, orange_ellipt_v, orange_aux_v)*-blue_g.x + Get_g1y_Vec(orange_local_v, orange_ellipt_v, orange_aux_v)*blue_g.y + Get_g1z_Vec(orange_local_v, orange_ellipt_v, orange_aux_v)*blue_g.z
				orange_g1_orange <- Get_g1x_Vec(orange_local_v, orange_ellipt_v, orange_aux_v)*orange_g.x + Get_g1y_Vec(orange_local_v, orange_ellipt_v, orange_aux_v)*orange_g.y + Get_g1z_Vec(orange_local_v, orange_ellipt_v, orange_aux_v)*orange_g.z
				orange_local_acc <- orange_local_acc + (orange_g1_orange + orange_g1_blue)

				blue_acc <- Matrix_Vector_Mult_3D(blue_local, blue_local_acc)
				orange_acc <- Matrix_Vector_Mult_3D(orange_local, orange_local_acc)
				
				
				
				if(i == -1) {
					player.SetVelocity(player.GetVelocity() + (blue_acc + orange_acc)*dt);
					continue;
				}

				dir <- (blue_acc + orange_acc);
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
	
	function Generate() {
		blue_portal <- blue_temp;
		blue_origin <- blue_temp.GetOrigin();
		blue_local <- blue_temp_local;
		blue_local_inv <- Vectors_To_Matrix_Inv(blue_temp.GetUpVector(), blue_temp.GetLeftVector(), blue_temp.GetForwardVector());
		orange_portal <- orange_temp;
		orange_origin <- orange_temp.GetOrigin();
		orange_local <- orange_temp_local;
		orange_local_inv <- Vectors_To_Matrix_Inv(orange_temp.GetUpVector(), orange_temp.GetLeftVector(), orange_temp.GetForwardVector());
		
		blue_g <- Matrix_Vector_Mult_3D(blue_local_inv, Vector(0.0,0.0,-g));
		orange_g <- Matrix_Vector_Mult_3D(orange_local_inv, Vector(0.0,0.0,-g));
		
		blue_orange_local_v <- Matrix_Vector_Mult_3D(orange_local_inv, blue_origin - orange_origin)
		blue_orange_ellipt_v <- Get_Ellipsoidal_Coords(blue_orange_local_v);
		orange_blue_local_v <- Matrix_Vector_Mult_3D(blue_local_inv, orange_origin - blue_origin)
		orange_blue_ellipt_v <- Get_Ellipsoidal_Coords(orange_blue_local_v);
		
		local d0; 
		if(blue_orange_ellipt_v.x > 0)
			d0 = 1.0/(Integral(0.0, atan(blue_orange_ellipt_v.x/b_), E_0, 128) + Integral(0.0, atan(blue_orange_ellipt_v.x/b_), E_0, 128));
		else
			d0 = 1.0;
		dh <- d0*g*(orange_origin - blue_origin).z;
		
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
		local max_dist = -1.0;
		local max_index = -1;
		
		for(local i = 0; i < names.len(); i++){
			foreach(origin in [blue_origin, orange_origin]) {
				if (props.len() < max_props) {
					curr <- Entities.FindByClassname(null, names[i])
				} else {
					max_dist = -1.0;
					max_index = -1;
					for(local i = 0; i<props.len(); i++) {
						local prop_origin = Entities.FindByName(null, props[i]).GetOrigin();
						local dist = min((prop_origin-blue_origin).Length(), (prop_origin-orange_origin).Length());
						if(dist > max_dist) {
							max_dist = dist;
							max_index = i;
						}
					}
					curr <- Entities.FindByClassnameWithin(null, names[i], origin, max_dist);
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
								local prop_origin = Entities.FindByName(null, props[i]).GetOrigin();
								local dist = min((prop_origin-blue_origin).Length(), (prop_origin-orange_origin).Length());
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
						curr <- Entities.FindByClassnameWithin(curr, names[i], origin, max_dist);
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
	
	// Check the existence of portals
	if(!detected_blue) {
		blue_portal <- null;
		orange_portal <- null;
		return;
	}
	
	blue_temp <- Entities.FindByClassnameNearest("prop_portal", detected_blue, 1.0);
	orange_temp <- Entities.FindByClassnameNearest("prop_portal", detected_orange, 1.0);
	::detected_blue <- null;
	::detected_orange <- null;
	
	// If portals moved this frame - skipping
	if(!blue_temp || !orange_temp) {
		Check_Remove();
		Check_Add();
		return;
	}
	
	// Generate coefficients and run physics for newly created portals
	if(!blue_portal || !orange_portal) {
		blue_temp_local <- Vectors_To_Matrix(blue_temp.GetUpVector(), blue_temp.GetLeftVector(), blue_temp.GetForwardVector());
		orange_temp_local <- Vectors_To_Matrix(orange_temp.GetUpVector(), orange_temp.GetLeftVector(), orange_temp.GetForwardVector());
		Generate();
		Check_Remove();
		Check_Add();
		
		Run_Physics();
		return;
	}
	
	// Generate coefficients and run physics for replaced portals
	blue_temp_local <- Vectors_To_Matrix(blue_temp.GetUpVector(), blue_temp.GetLeftVector(), blue_temp.GetForwardVector());
	orange_temp_local <- Vectors_To_Matrix(orange_temp.GetUpVector(), orange_temp.GetLeftVector(), orange_temp.GetForwardVector());
	if (!Vector_Eq(blue_origin, blue_temp.GetOrigin()) || !Matrix_Eq(blue_local, blue_temp_local) || !Vector_Eq(orange_origin, orange_temp.GetOrigin()) || !Matrix_Eq(orange_local, orange_temp_local)) {
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
