struct Data
{
    //gps
    uint32_t UNIX_time;
    double lat, lon, alt, speed/*?*/, course/*?*/;

    //mpu6050
    double ax, ay, az, wx, wy, wz;

    //bmp390
    double pressure, temp;
};

Data data;
int last_glide;

void setup(){
    inti sensors;
        //put in sleep mode

    while(1){
        check lunch condition (use Jam Jar one);
        if(launched){ ////////////state///////////////
            break;
        }
    }

    while(launched && !deployed){ /////////////state/////////////
        get data;
        store data;
    }
    


}

void loop(){ //main algorthim
    // add glide time routine, can use the unix time to calculate
    get_data();

    current_course = gps.course.deg();
    ideal_course = TingGPSPlus.courseTo(lat, lon, target_lat, target lon);

    if((data.UNIX_time - last_glide_time) > glide_interval){
        algorthim;
        spin_motor();
        glide;
    }

    store_data();


}