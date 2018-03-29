/**
* New in this version: Active trackers with no measurements within their chi squared threshold
  are not considered in the Hungarian algorithm, so valid tracks are not randomly kicked out.
*/

// ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
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

// ROS Publishers
  ros::Publisher object_path_pub;
  ros::Publisher object_path_estimate_pub;
  ros::Publisher ellipse_pub;
  ros::Publisher text_pub;
  ros::Publisher radius_pub;

// Kalman filter parameters
  int n = 4; // Number of states
  int m = 2; // Number of measurements

  double std_accel = 0.1; // Process noise (acceleration)
  double std_pos   = 0.1; // Measurement nosie (position)

  double dt = 1.0/10; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd Bw(n, 1); // System dynamics matrix
  Eigen::MatrixXd H(m, n); // Output matrix
  // Eigen::MatrixXd Q(1, 1); // Process noise covariance
  Eigen::MatrixXd Qd(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

// Estimated object centers point cloud container
  // pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>());

// Tracking parameters
  int max_measurements = 20;   // max # of measurements to consider
  double effective_radius = 10;   // radius in which to look for max # of measurements
  double effective_radius2 = 1; // inside radius
  double effective_height = 1;  // search height for object centers
  int association_limit = 3;  // # of times to acquire consecutive associations before tracker is invalidated on first dissociation
  int dissociation_limit = 3;
  int max_trackers = max_measurements * dissociation_limit; // maximum number of trackers to use
  double chi_thresh = 5.99;

  std::vector<Tracker> trackers;  // vector of tracker objects

// Timing
  ros::Time dt_begin;


void measurement_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  // ROS timing
  ros::Time begin = ros::Time::now();
  dt = ros::Time::now().toSec() - dt_begin.toSec();
  dt_begin = ros::Time::now();


  printf("Callback function\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects_estimate(new pcl::PointCloud<pcl::PointXYZ>());
  // Get measurements within search radius
    std::vector<int> object_inlier_indices;
    std::vector<double> object_ranges;
    for (int i = 0; i < msg->points.size(); i++)
    {
      double range = sqrt(pow(msg->points[i].x,2) + pow(msg->points[i].y,2));
      object_ranges.push_back(range);
      double height = msg->points[i].z; // Object center height
      if (range <= effective_radius && range >= effective_radius2 && height <= effective_height)
      {
        object_inlier_indices.push_back(i);
        pcl::PointXYZ point;
        point.x = msg->points[i].x;
        point.y = msg->points[i].y;
        point.z = 0;
        objects->push_back(point);
      }
    }

    int num_measurements = objects->points.size();
    objects->header.frame_id = "velodyne";
    object_path_pub.publish(objects);
    printf("Objects within range = %d\n", num_measurements);

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

  // Algorithm takes two paths here: 1-Try to pair measuremetents with active trackers
  //                                 2-Initialize trackers if none are available
    if (active_tracker_indices.size() == 0)
    {
      // Initialize same number of trackers as measurements
      // int num_measurements = objects->points.size();
      for (int i = 0; i < num_measurements; i++)
      {
        Eigen::VectorXd x(n);
        x(0) = objects->points[i].x;
        x(1) = objects->points[i].y;
        x(2) = vvx;
        x(3) = vvy;
        trackers[i].init(ros::Time::now().toSec(), x);
      }
      printf("Initialized %d tracking channels\n", num_measurements);

    }
    else
    {
      // printf("association loop.....\n");
      // printf("run time update for active trackers\n");
      /**
      *  If active trackers are available, try to associate measurements to those trackers first.
      *  Measurements are associated using Hungarian algorithm.
      */
        // int num_measurements = objects->points.size();
        int num_active_trackers = active_tracker_indices.size();
        std::vector<int> passed_active_tracker_indices;
        std::vector<std::vector<double> > scores_pass;  // Temporary container for chi score columns
        std::vector<std::vector<double> > scores_pass2;  // Temporary container for chi score columns
        std::vector<double> min_score_indices; // Indices of smalles scores per tracker

        for (int i = 0; i < num_active_trackers; i++)
        {
          int pass_flag = 0; // This flag is 1 if value in the tracker's column passes the chi threshold
          std::vector<double> score_column; // Save the chi scores for current tracker
          score_column.resize(num_measurements);
          trackers[active_tracker_indices[i]].timeUpdate(dt);  // run time update on active trackers
          double min_score = 2.0*chi_thresh;  // minimum score per column of measurements
          int min_score_index = 0;

          for (int j = 0; j < num_measurements; j++)
          {
            Eigen::VectorXd y(m);
            y(0) = objects->points[j].x;
            y(1) = objects->points[j].y;
            score_column[j] = trackers[active_tracker_indices[i]].getMahalanobis(y); // get chi squared score

            if (score_column[j] <= chi_thresh)
            {
              pass_flag = 1;
              if (score_column[j] < min_score)  // Check minimum score
              {
                min_score = score_column[j];
                min_score_index = j;
              }
            }
          }

          // If column passed, then add to chi scores matrix
          if (pass_flag == 1)
          {
            // Add column to scores array
              scores_pass.push_back(score_column);

            // Add tracker index to passed active tracker indices vector
              passed_active_tracker_indices.push_back(active_tracker_indices[i]);

            // Add minimum score to minimum scores vector
              min_score_indices.push_back(min_score_index);
          }
          else
          {
            // Increment dissociation of tracker corresponding to failed column of measurements
              trackers[active_tracker_indices[i]].incDissociation();
          }
        }

        int num_passed_trackers = scores_pass.size(); // Number of trackers that passed
        printf("Number of chi2-passed trackers = %d\n", num_passed_trackers);

        // Check for case of trackers > measurements, and pass through only (x=measurements) trackers with best scors

          // if (num_passed_trackers > num_measurements)
          // {
          //
          // }
        // Check for repeated minimum score indices (each tracker has unique measurement pair)
          int multi_pair_flag = 0;  // Flag to catch pairing of measurement to multiple trackers

          for (int i = 0; i < min_score_indices.size(); i++)
          {
            for (int j = 0; j < min_score_indices.size(); j++)
            {
              if (i != j)
              {
                if (min_score_indices[i] == min_score_indices[j])
                {
                  multi_pair_flag = 1;
                }
              }
            }
            if (multi_pair_flag)
            {
              break;
            }
          }




        // If only unique tracker-measurement pairs, use minimum scores
          std::vector<Estimate> current_estimates;        // vector of structs holding estimates at this time step
          std::vector<int> unpaired_measurements_indices; // vector of indices of unpaired measurements
          std::vector<int> new_measurement_indices;


          if (!multi_pair_flag)
          {
            printf("REGULAR Association Used\n");

            // Print tracker ID - Score pairs
              for (int i = 0; i < num_passed_trackers; i++)
              {
                printf("tracker ID %d score = %10.4f\n", trackers[passed_active_tracker_indices[i]].getID(), scores_pass[i][min_score_indices[i]]);
              }
            // Find indices of measurements that were not paired
              for (int i = 0; i < num_measurements; i++)
              {
                int found_measurement_flag = 0;
                for (int j = 0; j < num_passed_trackers; j++)
                {
                  if (i == min_score_indices[j])
                  {
                    found_measurement_flag++;
                  }
                }
                if (!found_measurement_flag)
                {
                  unpaired_measurements_indices.push_back(i);
                }
              }

            // Measurement update for trackers
            for (int i = 0; i < num_passed_trackers; i++)
            {
              // Increment association for trackers
                trackers[passed_active_tracker_indices[i]].incAssociation();

              // Run measurement update for validated pair
                Estimate estimate_pair;
                Eigen::VectorXd y(m);
                y(0) = objects->points[min_score_indices[i]].x;
                y(1) = objects->points[min_score_indices[i]].y;
                estimate_pair.x_estimated = trackers[passed_active_tracker_indices[i]].measurementUpdate(y);
                estimate_pair.tracker_ID = trackers[passed_active_tracker_indices[i]].getID();
                estimate_pair.measurement_ID = min_score_indices[i];
                Eigen::VectorXd innovation_ellipse = trackers[passed_active_tracker_indices[i]].getEllipse(chi_thresh);
                estimate_pair.major_axis_length = innovation_ellipse(0);
                estimate_pair.minor_axis_length = innovation_ellipse(1);
                estimate_pair.angle = innovation_ellipse(2);

                current_estimates.push_back(estimate_pair);

            }

            // For compatibility only
            for (int i = 0; i < num_measurements; i++)
            {
              new_measurement_indices.push_back(i);
            }
          }
          else
          {
            printf("MUNKRES Association Used\n");
            // Remove outlier measurements to avoid mismatching
              for (int i = 0; i < num_measurements; i++)
              {
                int measurement_inlier_flag = 0;  // set to 1 if measurement is inlier to at least 1 tracker
                for (int j = 0; j < num_passed_trackers; j++)
                {
                  if (scores_pass[j][i] <= chi_thresh)
                  {
                    measurement_inlier_flag = 1;
                  }
                }
                if (!measurement_inlier_flag)
                {
                  // Add measurement index to unpaired indices
                  unpaired_measurements_indices.push_back(i);
                }
                else
                {
                  // Add measurement index to new set of valid indices
                  new_measurement_indices.push_back(i);

                }
              }

  printf("here1\n");
            // Populate new scores matrix to reflect scores with valid indices
              num_measurements = new_measurement_indices.size();
              // for (int i = 0; i < num_measurements; i++)
              // {
              //   for (int j = 0; j < num_passed_trackers; j++)
              //   {
              //     scores_pass2[j][i] = new_measurement_indices[i];
              //   }
              // }
              // scores_pass2.resize(num_passed_trackers);
              for (int i = 0; i < num_passed_trackers; i++)
              {
                // scores_pass2[i].resize(num_measurements);
                std::vector<double> temp_scores;
                for (int j = 0; j < num_measurements; j++)
                {
                  temp_scores.push_back(scores_pass[i][new_measurement_indices[j]]);
                  // printf("score = %f\n", scores_pass[i][new_measurement_indices[j]]);
                  // scores_pass2[i].push_back(scores_pass[i][new_measurement_indices[j]]);
                }
                scores_pass2.push_back(temp_scores);
              }

            // Populate chi squared scores matrix for Munkres algorithm
              Matrix<double> chi2_scores(num_measurements, num_passed_trackers);
              Matrix<double> munkres_scores(num_measurements, num_passed_trackers);

              for (int i = 0; i < num_measurements; i++)
              {
                for (int j = 0; j < num_passed_trackers; j++)
                {
                  // chi2_scores(i,j) = scores_pass[j][i];
                  chi2_scores(i,j) = scores_pass2[j][i];
                }
              }

  printf("here2\n");

          // If no trackers passed (tracked objects move out of scope), then do not go further
  printf("num_measurements = %d\n", num_measurements);
            munkres_scores = chi2_scores; // copy scores (since algorithm uses pointer to return values to original matrix)

            if (num_passed_trackers > 0)
            {
              // printf("display chi2 scores\n");
              // Use Hungarian algorithm to find associations
              // Row indices are measurement indices, and column indices are active tracker indices

                // Display chi2 scores matrix.
                for ( int row = 0 ; row < num_measurements ; row++ ) {
                  for ( int col = 0 ; col < num_passed_trackers ; col++ ) {
                    std::cout.width(2);
                    std::cout << chi2_scores(row,col) << ",";
                  }
                  std::cout << std::endl;
                }

              // printf("run hungarian algorithm on scores\n");
              Munkres<double> munkres_algorithm;
              munkres_algorithm.solve(munkres_scores);

                // Display solved Munkres matrix.
                for ( int row = 0 ; row < num_measurements ; row++ ) {
                  for ( int col = 0 ; col < num_passed_trackers ; col++ ) {
                    std::cout.width(2);
                    std::cout << munkres_scores(row,col) << ",";
                  }
                  std::cout << std::endl;
                }


              // printf("find associations\n");

  printf("here3\n");

                for ( int row = 0 ; row < num_measurements ; row++ )
                {
                  int rowcount = 0; // Number of zeros in each row of scores
                  Estimate estimate_pair;
                  for ( int col = 0 ; col < num_passed_trackers ; col++  )
                  {
                    if ( munkres_scores(row,col) == 0 )
                    {
                      rowcount++;
                      // Check if measurement falls inside validation region of tracker
                        if (trackers[passed_active_tracker_indices[col]].isValid(chi2_scores(row,col), chi_thresh))
                        {
                          // Tracker's association variable has been incremented implicitly
                          // Run measurement update for validated pair
                            Eigen::VectorXd y(m);
                            y(0) = objects->points[new_measurement_indices[row]].x;
                            y(1) = objects->points[new_measurement_indices[row]].y;
                            estimate_pair.x_estimated = trackers[passed_active_tracker_indices[col]].measurementUpdate(y);
                            estimate_pair.tracker_ID = trackers[passed_active_tracker_indices[col]].getID();
                            estimate_pair.measurement_ID = new_measurement_indices[row];
                            Eigen::VectorXd innovation_ellipse = trackers[passed_active_tracker_indices[col]].getEllipse(chi_thresh);
                            estimate_pair.major_axis_length = innovation_ellipse(0);
                            estimate_pair.minor_axis_length = innovation_ellipse(1);
                            estimate_pair.angle = innovation_ellipse(2);

                            current_estimates.push_back(estimate_pair);

                        }
                        else
                        {
                          // Tracker's dissociation variable has been incremented implicitly
                          // add measurement index to unpaired measurements vector
                            unpaired_measurements_indices.push_back(new_measurement_indices[row]);
                        }

                    }

                  }
                  if ( rowcount != 1 )
                  {
                    // Since rows represent measurements, empty rows are unpaired measurements.
                    // Add measurement to unpaired measurements vector
                    unpaired_measurements_indices.push_back(new_measurement_indices[row]);
                    // std::cerr << "Row " << row << " has " << rowcount << " columns that have been matched." << std::endl;

                  }
                }
              // printf("current_estimates = %d, unpaired_measurements = %d\n", (int)current_estimates.size(), (int)unpaired_measurements_indices.size());
printf("here4\n");
              // Increment dissociation of active trackers that were not associated (columns without zeros)
                for (int i = 0; i < num_passed_trackers; i++)
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
                    trackers[passed_active_tracker_indices[i]].incDissociation();
                  }
                }
            }
            else
            {
              // If no trackers passed, then their dissociation was already incremented above

printf("here5\n");
              // If there are any unassociated measurements that arrive, hand them to new trackers
                for (int i = 0; i < num_measurements; i++)
                {
                  unpaired_measurements_indices.push_back(new_measurement_indices[i]);
                }
            }

      }

      // printf("Update list of active/inactive trackers\n");
      // Initialize inactive trackers for remaining unpaired measurements
        // Get list of inactive trackers
          std::vector<int> inactive_tracker_indices;
          std::vector<int> new_active_tracker_indices;

          for (int i = 0; i < max_trackers; i++)
          {
            if (!trackers[i].isActive())
            {
              inactive_tracker_indices.push_back(i);
            }
            else
            {
              new_active_tracker_indices.push_back(i);
            }
          }
        printf("  active_trackers = %d inactive_trackers = %d\n", (int)new_active_tracker_indices.size(), (int)inactive_tracker_indices.size());

printf("here6\n");
        // printf("Initialize one tracker for each remaining measurement\n");
        // Initialize one tracker for each unpaired measurement for as many trackers available
          // if (unpaired_measurements_indices.size() <= inactive_tracker_indices.size())
          // {
          //
          //
          //   for (int i = 0; i < unpaired_measurements_indices.size(); i++)
          //   {
          //     Eigen::VectorXd x(n);
          //     x(0) = objects->points[unpaired_measurements_indices[i]].x;
          //     x(1) = objects->points[unpaired_measurements_indices[i]].y;
          //     x(2) = vvx;
          //     x(3) = vvy;
          //     trackers[inactive_tracker_indices[i]].init(ros::Time::now().toSec(), x);
          //
          //     Estimate estimate_pair;
          //     estimate_pair.x_estimated = trackers[inactive_tracker_indices[i]].getState();
          //     estimate_pair.tracker_ID = trackers[inactive_tracker_indices[i]].getID();
          //     estimate_pair.measurement_ID = unpaired_measurements_indices[i];
          //     Eigen::VectorXd innovation_ellipse = trackers[inactive_tracker_indices[i]].getEllipse(chi_thresh);
          //     estimate_pair.major_axis_length = innovation_ellipse(0);
          //     estimate_pair.minor_axis_length = innovation_ellipse(1);
          //     estimate_pair.angle = innovation_ellipse(2);
          //
          //     current_estimates.push_back(estimate_pair);
          //   }
          // }

          // Find number of needed trackers
            int num_needed_trackers = unpaired_measurements_indices.size();

            if (unpaired_measurements_indices.size() > inactive_tracker_indices.size())
            {
              num_needed_trackers = inactive_tracker_indices.size();
            }

printf("here7\n");
          for (int i = 0; i < num_needed_trackers; i++)
          {
            Eigen::VectorXd x(n);
            x(0) = objects->points[unpaired_measurements_indices[i]].x;
            x(1) = objects->points[unpaired_measurements_indices[i]].y;
            x(2) = vvx;
            x(3) = vvy;
            trackers[inactive_tracker_indices[i]].init(ros::Time::now().toSec(), x);

            Estimate estimate_pair;
            estimate_pair.x_estimated = trackers[inactive_tracker_indices[i]].getState();
            estimate_pair.tracker_ID = trackers[inactive_tracker_indices[i]].getID();
            estimate_pair.measurement_ID = unpaired_measurements_indices[i];
            Eigen::VectorXd innovation_ellipse = trackers[inactive_tracker_indices[i]].getEllipse(chi_thresh);
            estimate_pair.major_axis_length = innovation_ellipse(0);
            estimate_pair.minor_axis_length = innovation_ellipse(1);
            estimate_pair.angle = innovation_ellipse(2);

            current_estimates.push_back(estimate_pair);
          }

        printf("Total active trackers = %d\n", (int)current_estimates.size());

        // printf("Update innovation error ellipses \n");
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
        //   objects_estimate.reset(new pcl::PointCloud<pcl::PointXYZ>());

        // Publish ellipse marker for each tracker
          visualization_msgs::Marker search_radius;
          search_radius.header.frame_id = "velodyne";
          search_radius.header.stamp = ros::Time::now();
          search_radius.ns = "search_radius";
          search_radius.type = visualization_msgs::Marker::CYLINDER;
          search_radius.action = visualization_msgs::Marker::ADD;
          search_radius.color.a = 0.1;
          search_radius.color.r = 0;
          search_radius.color.g = 1;
          search_radius.color.b = 0;
          // search_radius.lifetime = ros::Duration(0.1);
          search_radius.id = 0;
          search_radius.pose.position.x = 0;
          search_radius.pose.position.y = 0;
          search_radius.pose.position.z = 0;
          // search_radius.pose.orientation.x = qt.x();
          // search_radius.pose.orientation.y = qt.y();
          // search_radius.pose.orientation.z = qt.z();
          // search_radius.pose.orientation.w = qt.w();
          search_radius.scale.x = 2*effective_radius;
          search_radius.scale.y = 2*effective_radius;
          search_radius.scale.z = 0.01;

          radius_pub.publish(search_radius);
          // Ellipse and text marker arrays
            visualization_msgs::MarkerArray ellipse_array;
            visualization_msgs::MarkerArray text_array;


          for (int i = 0; i < current_estimates.size(); i++)
          {
            // Ellipse marker
              visualization_msgs::Marker ellipse;
              ellipse.header.frame_id = "velodyne";
              ellipse.header.stamp = ros::Time::now();
              ellipse.ns = "tracker_ellipses";
              ellipse.type = visualization_msgs::Marker::CYLINDER;
              ellipse.action = visualization_msgs::Marker::ADD;
              ellipse.color.a = 0.5;
              ellipse.color.r = 0;
              ellipse.color.g = 0;
              ellipse.color.b = 1;
              ellipse.lifetime = ros::Duration(dt);

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

              ellipse_array.markers.push_back(ellipse);

            // Text marker
              visualization_msgs::Marker text;
              text.header.frame_id = "velodyne";
              text.header.stamp = ros::Time::now();
              text.ns = "tracker_IDs";
              text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
              text.action = visualization_msgs::Marker::ADD;
              text.color.a = 1;
              text.color.r = 1;
              text.color.g = 1;
              text.color.b = 1;
              text.lifetime = ros::Duration(dt);

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
              text.scale.z = 1;

              text_array.markers.push_back(text);

              // ellipse_pub.publish(ellipse);
              // text_pub.publish(text);
          }

          ellipse_pub.publish(ellipse_array);
          text_pub.publish(text_array);

    }

  // Timer
    double loop_time = ros::Time::now().toSec() - begin.toSec();
    printf("loop_time = %5.3f, dt = %5.3f\n", loop_time, dt);
    printf("\n");



}


int main(int argc, char* argv[]) {

  // ROS initialization
    ros::init (argc, argv, "tracking_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);  //10 hz simulation

    // Subscribe to object measurement point cloud
    ros::Subscriber measurement_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("object_cloud", 1, measurement_cb);
    // Publish actual object path
    object_path_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("object_path", 1);
    // Publish estimated object path
    object_path_estimate_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("object_path_estimate", 1);
    // Publish ellipse markers
    ellipse_pub = nh.advertise<visualization_msgs::MarkerArray> ("ellipse", 1);
    // Publish text markers
    text_pub = nh.advertise<visualization_msgs::MarkerArray> ("text", 1);
    // Publish radius marker
    radius_pub = nh.advertise<visualization_msgs::Marker> ("search_radius", 1);

    // Discrete LTI projectile motion, measuring position only
      A << 1, 0, dt, 0,
           0, 1, 0, dt,
           0, 0, 1, 0,
           0, 0, 0, 1;

      H << 1, 0, 0, 0,
           0, 1, 0, 0;

      Bw << 0.5*pow(dt,2),
            0.5*pow(dt,2),
            dt,
            dt;

      // Covariance matrices
      // Q << pow(std_accel,2);

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

   // Initialize trackers
     for (int i = 0; i < max_trackers; i++)
     {
       Tracker tracker(association_limit, dissociation_limit, A, Bw, H, Qd, R, P);
       tracker.setID(i);
       trackers.push_back(tracker);
     }
     printf("Max trackers available = %d\n\n", max_trackers);


   // Spin and timing initiate
   dt_begin = ros::Time::now();

   ros::spin();
}
