#include "ros/ros.h"
#include "math.h"

class GPS_Odometer{
private:
	ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;

    double x, y, z; // valori del robot

    void gps_to_odom(double lat_r, double lon_r, double alt_r){ // i parametri sono di riferimento
    	double X, Y, Z, X_r, Y_r, Z_r, N, N_r, e2;
    	// [lat, lon, alt] = getRobotPosition() // ottenere i valori dal GPS del robot
    	e2 = 1 - 6356752^2/6378137^2;
    	N_r = 6378137 / (sqrt(1 - e2 * (sin(lat_r))^2));
    	N = 6378137 / (sqrt(1 - e2 * (sin(lat))^2));

    	// from GPS to ECEF
    	X_r = (N_r + alt_r) * cos(lat_r) * cos(lon_r);
    	X = (N + alt) * cos(lat) * cos(lon);
    	Y_r = (N_r + alt_r) * cos(lat_r) * sin(lon_r);
    	Y = (N + alt) * cos(lat) * sin(lon);
    	Z_r = (N_r * (1 - e2) + alt_r) * sin(lat_r);
    	Z = (N * (1 - e2) + alt) * sin(lat);

    	// controllare se i valori sono in radianti o gradi
    	// phi = lat, lambda = lon, h = altitude
    	// from ECEF to ENU
    	double matrix[3][3] = {
    		{-sin(lon_r), cos(lon_r), 0},
    		{-sin(lat_r) * cos(lon_r), -sin(lat_r) * sin(lon_r), cos(lat_r)},
    		{cos(lat_r) * cos(lon_r), cos(lat_r) * sin(lon_r), sin(lat_r)}
    	};
    	double column[3] = {X - X_r, Y - Y_r, Z - Z_r};
    	// [x, y, z] = matrix_multiply(matrix, column); trovare funzione per moltiplicazione matriciale/vettoriale
    }

public:
	void init(){

	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "gps_odometer");
	GPS_Odometer gps_odometer;
	gps_odometer.init();
	return 0;
}