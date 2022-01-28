//helper functions for finding the direction of the up vector 
bool xDir (const Vec3& up_vec) { return (up_vec.x ? 1 : 0); }
bool yDir (const Vec3& up_vec) { return (up_vec.y ? 1 : 0); }
bool zDir (const Vec3& up_vec) { return (up_vec.z ? 1 : 0); }

TrajectoryResult PredictTrajectory( const Vec3& start_position, const Vec3& start_velocity, const Vec3& up_vector, float gravity_accel, float raycast_time_step, float max_time )
{
    //constants
    
    //returns the direction of the up vector
    char xdir ='n';
    char ydir ='n';
    char zdir ='n';
    if (xDir(up_vector)) {xdir = 'x';}
    if (yDir(up_vector)) {ydir = 'y';}
    if (zDir(up_vector)) {zdir = 'z';}
    
    bool hit = false;
    bool d1 = false;
    bool max = false;
    float start_coord = 0.0;
    float distance1 = 0.0;
    float distance2 = 0.0;
    float end_coord = 0.0;
    float velocity_mag = 0.0;
    float up_mag = Sqrtf(pow(up_vector.x,2) + pow(up_vector.y,2) + pow(up_vector.z,2)); //unused but I'm sure that it has to be used somewhere 
    
    Vec3 curr_pos = start_position;
    Vec3 prev_pos = start_position;
    Vec3 gravity_f = up_vector * gravity_accel; //gravity vector set to the opposite direction as the up_vector
    
    TrajectoryResult tResult; //instantiate a TrajectoryResult object
           
    float i;
    for (i = 0.0; i < max_time; i += raycast_time_step)
    {
        // calculate current position based on the kinematics equation:
        // (x,y,z) = initial velocity_mag * time + 0.5 * gravity * time^2
        curr_pos = start_velocity * i + gravity_f * 0.5 * i * i + start_position;
           
        if (Physics::Raycast(prev_pos, curr_pos).m_ValidHit) // successful hit
        {
            float gravity_mag = gravity_accel;
            Vec3 hit_pos = Physics::Raycast(prev_pos, curr_pos).m_HitPos;
            Vec3 vert_travel = curr_pos * up_vector - prev_pos * up_vector; // calculate the vertical distance
               
            //hit_pos * up_vector isolates the vertical position of the target
            Vec3 distance_vert = hit_pos * up_vector - vert_travel; //store the vertical distance
            
            Vec3 distance_geo = up_vector * Sqrtf(pow(Physics::Raycast(prev_pos, curr_pos).m_HitPos.x - curr_pos.x, 2) 
                                + pow(hit_pos.y - curr_pos.y, 2) 
                                + pow(hit_pos.z - curr_pos.z, 2)); //store the geometric distance
            
            //store the appropriate values corresponding to the direction of the up vector
            if (xdir == 'x')
            {
                distance1 = distance_vert.x;
                distance2 = distance_geo.x;
                velocity_mag = start_velocity.x;
                start_coord = start_position.x;
                end_coord = hit_pos.x;
            }
            
            if (ydir == 'y')
            {
                distance1 = distance_vert.y;
                distance2 = distance_geo.y;
                velocity_mag = start_velocity.y;
                start_coord = start_position.y;
                end_coord = hit_pos.y;
            }
            
            if (zdir == 'z')
            {
                distance1 = distance_vert.z;
                distance2 = distance_geo.z;
                velocity_mag = start_velocity.z;
                start_coord = start_position.z;
                end_coord = hit_pos.z;
            }
            
            if (xdir == 'x' && ydir == 'y' && zdir == 'z')
            {
                gravity_mag = Sqrtf(pow(gravity_f.x, 2) + pow(gravity_f.y, 2) + pow(gravity_f.z, 2));

                Vec3 tempVec = Physics::Raycast(prev_pos, curr_pos).m_HitPos;
    
                Physics::Plane hit_plane( tempVec , 0.0f ); //create and add a plane
                Physics::AddPlane( hit_plane);

                //calculate the two distances
                //distance1 == distance from point to point
                //distance2 == distance from point to plane
                distance1 = Sqrtf(pow(Physics::Raycast(prev_pos, curr_pos).m_HitPos.x - curr_pos.x, 2)
                                  + pow(Physics::Raycast(prev_pos, curr_pos).m_HitPos.y - curr_pos.y, 2)
                                  + pow(Physics::Raycast(prev_pos, curr_pos).m_HitPos.z - curr_pos.z, 2));
                   
                distance2 = Absf(-1* hit_plane.normal.x * curr_pos.x - hit_plane.normal.y * curr_pos.y - hit_plane.normal.z * curr_pos.z) / (Sqrtf( pow(hit_plane.normal.x,2) + pow(hit_plane.normal.y,2) + pow(hit_plane.normal.z,2) ));

                //calculate the magnitude of the velocity
                velocity_mag = Sqrtf(pow(start_velocity.x, 2) + pow(start_velocity.y, 2) + pow(start_velocity.z, 2));

                float norm_mag = Sqrtf( pow(hit_plane.normal.x, 2) + pow(hit_plane.normal.y, 2) + pow(hit_plane.normal.z, 2) ); 
                
                float d5 =  (Physics::Raycast(prev_pos, curr_pos).m_HitPos.x - start_position.x) * hit_plane.normal.x * up_vector.x
                            + (Physics::Raycast(prev_pos, curr_pos).m_HitPos.y - start_position.y) * hit_plane.normal.y * up_vector.y
                            + (Physics::Raycast(prev_pos, curr_pos).m_HitPos.z - start_position.z) * hit_plane.normal.z * up_vector.z;
                
                //calculate the total distance travelled
                end_coord = Sqrtf(pow(Physics::Raycast(prev_pos, curr_pos).m_HitPos.x - start_position.x , 2)
                                  + pow(Physics::Raycast(prev_pos, curr_pos).m_HitPos.y - start_position.y , 2)
                                  + pow(Physics::Raycast(prev_pos, curr_pos).m_HitPos.z - start_position.z , 2 ));

                start_coord = 0; 
                
            }
            
            if (distance1 < distance2) //distance2 is greater, so return the time of distance1
            {
                d1 = true;
                
                float time1 = ((-1 * velocity_mag + Sqrtf( pow(velocity_mag, 2)
                                + 2 * gravity_mag * ( end_coord -  start_coord))) / gravity_mag );
                   
                float time2 = ((-1 * velocity_mag - Sqrtf( pow(velocity_mag, 2)
                                + 2 * gravity_mag * ( end_coord - start_coord))) / gravity_mag);

                if (time1 <= 0) //if time1 or time2 are less than 0, (negative time) => return the other time
                {
                    tResult.m_Time = time2;
                }
                else if (time2 <= 0)
                {
                    tResult.m_Time = time1;
                }
                else if (time1 < time2) //otherwise => return the smallest time
                {
                    tResult.m_Time = time1;
                       
                }
                else tResult.m_Time = time2;
                
            }

            hit = true;
            curr_pos = Physics::Raycast(prev_pos, curr_pos).m_HitPos;
            
            tResult.m_ValidHit = hit;
            tResult.m_EndPoint = curr_pos;
            
            if (hit && !d1) {tResult.m_Time = i;}
            
            return tResult;
            
        }
        
        prev_pos = curr_pos;
        
        
        //checks if the hit is at max_time
        float j = i;
        if (j + raycast_time_step >= max_time)
        {
            Vec3 temp_pos = start_velocity * max_time + gravity_f * 0.5 * max_time * max_time + start_position;
            if ((Physics::Raycast(prev_pos, temp_pos).m_ValidHit)) { max = true;}
        }
    }
    
    //update tResult's values depending on our output
    tResult.m_ValidHit = hit;
    tResult.m_EndPoint = curr_pos;
       
    if (!hit || max){ //if we did not have a successful hit or we are at max time
        tResult.m_Time = max_time;
        tResult.m_EndPoint = start_velocity * max_time + gravity_f * 0.5 * max_time * max_time + start_position;
        if (max) { tResult.m_ValidHit = true; }
    }
    
    return tResult;
}
