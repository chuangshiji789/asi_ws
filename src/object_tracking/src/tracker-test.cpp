// ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>


// Hungarian algorithm
#include "munkres.h"
#include "adapters/boostmatrixadapter.h"

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <sstream>

#include "tracker.h"

// Structure to hold estimate info
struct Estimate {
  int tracker_ID;     // index of tracker
  int measurement_ID; // index of measurement
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::VectorXd x_estimated;
  double major_axis_length;
  double minor_axis_length;
  double angle;

};

int main(int argc, char* argv[]) {

  // ROS initialization
    ros::init (argc, argv, "tracking_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);  //10 hz simulation
    // Publish actual object path
    ros::Publisher object_path_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("object_path", 1);
    // Publish estimated object path
    ros::Publisher object_path_estimate_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("object_path_estimate", 1);
    // Publish estimator markers
    ros::Publisher ellipse_pub = nh.advertise<visualization_msgs::Marker> ("ellipse", 1);
    // Publish estimator markers
    ros::Publisher text_pub = nh.advertise<visualization_msgs::Marker> ("text", 1);

  // Kalman filter parameters
    int n = 4; // Number of states
    int m = 2; // Number of measurements

    double std_accel = 0.1; // Process noise (acceleration)
    double std_pos   = 0.1; // Measurement nosie (position)

    double dt = 1.0/10; // Time step

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd Bw(n, 1); // System dynamics matrix
    Eigen::MatrixXd H(m, n); // Output matrix
    Eigen::MatrixXd Q(1, 1); // Process noise covariance
    Eigen::MatrixXd Qd(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    // Discrete LTI projectile motion, measuring position only
    A << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;

    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    // Bw << pow(dt,2)/2,
    //       pow(dt,2)/2,
    //       dt,
    //       dt;
    Bw << 0.5*pow(dt,2),
          0.5*pow(dt,2),
          dt,
          dt;

    // Covariance matrices
    Q << pow(std_accel,2);
    // Qd = Bw*Q*Bw.transpose();

    // Qd = Bw*Bw.transpose()*pow(std_accel,2); // this way assumes that noise is constant between sampling times
    Qd << 0.1,   0,   0,   0,
          0  , 0.1,   0,   0,
          0  ,   0, 0.1,   0,
          0  ,   0,   0, 0.1;

    R << pow(std_pos,2), 0,
         0             , pow(std_pos,2);

    P << 0.1, 0, 0, 0,
         0, 0.1 , 0, 0,
         0, 0, 0.1, 0,
         0, 0, 0, 0.1;

   std::cout << "A: \n" << A << std::endl;
   std::cout << "H: \n" << H << std::endl;
   std::cout << "Qd: \n" << Qd << std::endl;
   std::cout << "R: \n" << R << std::endl;
   std::cout << "P: \n" << P << std::endl;

  // Object centers simulation (x1)
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_estimate(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointXYZ origin;
    origin.x = 0; origin.y = 0; origin.z = 0;
    objects->push_back(origin);
    origin.x = 0; origin.y = 2; origin.z = 0;
    objects->push_back(origin);

    double vel_x = 1; double vel_y = 0; double vel_z = 0;

  // Tracking parameters
    int max_measurements = 2;
    int association_limit = 3;
    int dissociation_limit = 3;
    int max_trackers = max_measurements * dissociation_limit; // maximum number of trackers to use
    printf("Max trackers available = %d\n\n", max_trackers);
    double chi_thresh = 5.99;

    std::vector<Tracker> trackers;  // vector of tracker objects
    for (int i = 0; i < max_trackers; i++)
    {
      Tracker tracker(association_limit, dissociation_limit, A, Bw, H, Qd, R, P);
      tracker.setID(i);
      trackers.push_back(tracker);
    }
    // std::vector<Tracker> trackers (max_trackers, Tracker(association_limit, dissociation_limit, A, Bw, Q, H, R, P));
    // std::vector<bool> tracker_states (max_trackers, 0); // vector of tracker states (active: true/false)


// Tracking loop
  while (ros::ok())
  {
    // Propagate simulated points
      objects->points[0].x +=  vel_x * dt;
      objects->points[0].y +=  vel_y * dt;
      objects->points[0].z +=  vel_z * dt;

      objects->points[1].x +=  vel_x * dt;
      objects->points[1].y +=  vel_y * dt;
      objects->points[1].z +=  vel_z * dt;
      // printf("Point = %5.2f, %5.2f, %5.2f\n", objects->points[0].x, objects->points[0].y, objects->points[0].z);
      objects->header.frame_id = "velodyne";
      object_path_pub.publish(objects);

      // Vehicle velocity
        double vvx = 0;
        double vvy = 0;



    // Create a vector with indices of active trackers
      std::vector<int> active_tracker_indices;
      for (int i = 0; i < max_trackers; i++)
      {
        // if (tracker_states[i] == true)
        // {
        //   active_tracker_indices.push_back(i);
        // }
        if (trackers[i].isActive())
        {
          active_tracker_indices.push_back(i);
        }
      }
      printf("active trackers = %d\n", (int)active_tracker_indices.size());
      printf("measurements received = %d\n", (int)objects->points.size());

    // Algorithm takes two paths here: 1-Try to pair measuremetents with active trackers
    //                                 2-Initialize trackers of none are available
      if (active_tracker_indices.size() == 0)
      {
        // Initialize same number of trackers as measurements
        int num_measurements = objects->points.size();
        for (int i = 0; i < num_measurements; i++)
        {
          Eigen::VectorXd x(n);
          x(0) = objects->points[i].x;
          x(1) = objects->points[i].y;
          x(2) = vvx;
          x(3) = vvy;
          trackers[i].init(ros::Time::now().toSec(), x);
        }
        printf("initiation loop.....\n");

      }
      else
      {
        printf("association loop.....\n");
        printf("run time update for active trackers\n");
        /**
        *  If active trackers are available, try to associate measurements to those trackers first.
        *  Measurements are associated using Hungarian algorithm.
        */
          int num_measurements = objects->points.size();
          int num_active_trackers = active_tracker_indices.size();
          // Eigen::MatrixXd chi2_scores(num_measurements, num_active_trackers); // matrix of chi squared scores
          Matrix<double> chi2_scores(num_measurements, num_active_trackers);
          Matrix<double> munkres_scores(num_measurements, num_active_trackers);

          for (int i = 0; i < num_active_trackers; i++)
          {
            trackers[active_tracker_indices[i]].timeUpdate(dt);  // run time update on active trackers
            for (int j = 0; j < num_measurements; j++)
            {
              Eigen::VectorXd y(m);
              y(0) = objects->points[j].x;
              y(1) = objects->points[j].y;
              chi2_scores(j,i) = trackers[active_tracker_indices[i]].getMahalanobis(y); // get chi squared score
            }
          }

        printf("display chi2 scores\n");
        // Use Hungarian algorithm to find associations
        // Row indices are measurement indices, and column indices are active tracker indices
          munkres_scores = chi2_scores; // copy scores (since algorithm uses pointer to return values to original matrix)

          // Display chi2 scores matrix.
          for ( int row = 0 ; row < num_measurements ; row++ ) {
            for ( int col = 0 ; col < num_active_trackers ; col++ ) {
              std::cout.width(2);
              std::cout << chi2_scores(row,col) << ",";
            }
            std::cout << std::endl;
          }

        printf("run hungarian algorithm on scores\n");
          Munkres<double> munkres_algorithm;
          munkres_algorithm.solve(munkres_scores);

          // Display solved Munkres matrix.
          for ( int row = 0 ; row < num_measurements ; row++ ) {
            for ( int col = 0 ; col < num_active_trackers ; col++ ) {
              std::cout.width(2);
              std::cout << munkres_scores(row,col) << ",";
            }
            std::cout << std::endl;
          }


        printf("find associations\n");
          std::vector<Estimate> current_estimates;        // vector of structs holding estimates at this time step
          std::vector<int> unpaired_measurements_indices; // vector of indices of unpaired measurements

          for ( int row = 0 ; row < num_measurements ; row++ )
          {
        		int rowcount = 0; // Number of zeros in each row of scores
            Estimate estimate_pair;
        		for ( int col = 0 ; col < num_active_trackers ; col++  )
            {
        			if ( munkres_scores(row,col) == 0 )
              {
                rowcount++;
                // Check if measurement falls inside validation region of tracker
                  if (trackers[active_tracker_indices[col]].isValid(chi2_scores(row,col), chi_thresh))
                  {
                    // Tracker's association variable has been incremented implicitly
                    // Run measurement update for validated pair
                      Eigen::VectorXd y(m);
                      y(0) = objects->points[row].x;
                      y(1) = objects->points[row].y;
                      estimate_pair.x_estimated = trackers[active_tracker_indices[col]].measurementUpdate(y);
                      estimate_pair.tracker_ID = trackers[active_tracker_indices[col]].getID();
                      estimate_pair.measurement_ID = row;
                      Eigen::VectorXd innovation_ellipse = trackers[active_tracker_indices[col]].getEllipse(chi_thresh);
                      estimate_pair.major_axis_length = innovation_ellipse(0);
                      estimate_pair.minor_axis_length = innovation_ellipse(1);
                      estimate_pair.angle = innovation_ellipse(2);

                      current_estimates.push_back(estimate_pair);

                  }
                  else
                  {
                    // Tracker's dissociation variable has been incremented implicitly
                    // add measurement index to unpaired measurements vector
                      unpaired_measurements_indices.push_back(row);
                  }

              }

        		}
        		if ( rowcount != 1 )
            {
              // Since rows represent measurements, empty rows are unpaired measurements.
              // Add measurement to unpaired measurements vector
              unpaired_measurements_indices.push_back(row);
              // std::cerr << "Row " << row << " has " << rowcount << " columns that have been matched." << std::endl;

            }
        	}
        printf("current_estimates = %d, unpaired_measurements = %d\n", (int)current_estimates.size(), (int)unpaired_measurements_indices.size());

        // Increment dissociation of active trackers that were not associated (columns without zeros)
          for (int i = 0; i < num_active_trackers; i++)
          {
            int zeros_in_column = 0;
            for (int j = 0; j < num_measurements; j++)
            {
              if (munkres_scores(j,i) == 0)
              {
                zeros_in_column++;
              }
            }

            if (zeros_in_column !=1)
            {
              trackers[active_tracker_indices[i]].incDissociation();
            }
          }

        printf("get list of inactive trackers\n");
        // Initialize inactive trackers for remaining unpaired measurements
          // Get list of inactive trackers
            std::vector<int> inactive_tracker_indices;

            for (int i = 0; i < max_trackers; i++)
            {
              if (!trackers[i].isActive())
              {
                inactive_tracker_indices.push_back(i);
              }
            }
          printf("number of inactive trackers = %d\n", (int)inactive_tracker_indices.size());

          std::vector<int> new_active_tracker_indices;

          for (int i = 0; i < max_trackers; i++)
          {
            if (trackers[i].isActive())
            {
              new_active_tracker_indices.push_back(i);
            }
          }
          printf("number of active trackers = %d\n", (int)new_active_tracker_indices.size());

          printf("initialize one tracker for each remaining measurement\n");
          // Initialize one tracker for each unpaired measurement IF trackers available
            if (unpaired_measurements_indices.size() <= inactive_tracker_indices.size())
            {


              for (int i = 0; i < unpaired_measurements_indices.size(); i++)
              {
                Eigen::VectorXd x(n);
                x(0) = objects->points[unpaired_measurements_indices[i]].x;
                x(1) = objects->points[unpaired_measurements_indices[i]].y;
                x(2) = vvx;
                x(3) = vvy;
                trackers[inactive_tracker_indices[i]].init(ros::Time::now().toSec(), x);
              }
            }

          printf("update innovation error ellipses \n");
          // // Update point cloud of estimates
          //   for (int i = 0; i < current_estimates.size(); i++)
          //   {
          //     pcl::PointXYZ point;
          //     point.x = current_estimates[i].x_estimated(0);
          //     point.y = current_estimates[i].x_estimated(1);
          //     point.z = 0;
          //
          //     objects_estimate->points.push_back(point);
          //   }
          //
          //   objects_estimate->header.frame_id = "velodyne";
          //   object_path_estimate_pub.publish(objects_estimate);
            objects_estimate.reset(new pcl::PointCloud<pcl::PointXYZ>());

          // Publish ellipse marker for each tracker
            visualization_msgs::Marker ellipse;
            ellipse.header.frame_id = "velodyne";
            ellipse.header.stamp = ros::Time::now();
            ellipse.ns = "tracker_ellipse";
            ellipse.type = visualization_msgs::Marker::CYLINDER;
            ellipse.action = visualization_msgs::Marker::ADD;
            ellipse.color.a = 0.5;
            ellipse.color.r = 0;
            ellipse.color.g = 0;
            ellipse.color.b = 1;

            visualization_msgs::Marker text;
            text.header.frame_id = "velodyne";
            text.header.stamp = ros::Time::now();
            text.ns = "tracker_text";
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.color.a = 1;
            text.color.r = 1;
            text.color.g = 1;
            text.color.b = 1;

            for (int i = 0; i < current_estimates.size(); i++)
            {
              tf::Quaternion qt;
              qt.setRPY(0, 0, current_estimates[i].angle);

              ellipse.id = current_estimates[i].tracker_ID;
              ellipse.pose.position.x = current_estimates[i].x_estimated(0);
              ellipse.pose.position.y = current_estimates[i].x_estimated(1);
              ellipse.pose.position.z = 0;
              ellipse.pose.orientation.x = qt.x();
              ellipse.pose.orientation.y = qt.y();
              ellipse.pose.orientation.z = qt.z();
              ellipse.pose.orientation.w = qt.w();
              ellipse.scale.x = current_estimates[i].major_axis_length;
              ellipse.scale.y = current_estimates[i].minor_axis_length;
              ellipse.scale.z = 0.01;

              std::string id_string;
              std::ostringstream temp;
              temp << current_estimates[i].tracker_ID;
              id_string = temp.str();

              text.text = id_string;
              text.id = current_estimates[i].tracker_ID;
              text.pose.position.x = current_estimates[i].x_estimated(0);
              text.pose.position.y = current_estimates[i].x_estimated(1);
              text.pose.position.z = 0;
              // text.pose.orientation.x = qt.x();
              // text.pose.orientation.y = qt.y();
              // text.pose.orientation.z = qt.z();
              // text.pose.orientation.w = qt.w();
              text.scale.x = current_estimates[i].major_axis_length;
              text.scale.y = current_estimates[i].minor_axis_length;
              text.scale.z = 2;

              ellipse_pub.publish(ellipse);
              text_pub.publish(text);
            }


      }

    printf("\n");
    loop_rate.sleep();
  }



}
