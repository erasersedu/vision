int64 id                                       #space number
bool isspace                                   #True if there is free space
float32[] frontleft                            #Metric front left centroid, [i_width, j_height, k_depth]
float32[] frontright                           #Metric front right centroid, [i_width, j_height, k_depth]
float32[] backleft                             #Metric back left centroid, [i_width, j_height, k_depth]
float32[] backright                            #Metric back right centroid, [i_width, j_height, k_depth]
sensor_msgs/PointCloud2 pcl                    #Point cloud
