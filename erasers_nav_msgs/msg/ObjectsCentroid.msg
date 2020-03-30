int64 id                                       #point number
bool isobject                                  #True if there are objects
int64 n                                        #Number of objects
int64 bb_w                                     #starting point vector width
int64 bb_h                                     #starting point vector height
float32[] bbox                                 #Bounding box, [x, y, width, height]
int64 cntr_w                                   #starting point vector width
int64 cntr_h                                   #starting point vector height
float32[] centroid                             #Metric centroid, [i_width, j_height, k_depth]
int64 evec_w                                   #starting point vector width
int64 evec_h                                   #starting point vector height
float32[] eigenvectors                         #Metric centroid, [i_width, j_height, k_depth]
int64 eval_w                                   #starting point vector width
int64 eval_h                                   #starting point vector height
float32[] eigenvalues                          #Object eigenvalues, [i_width, j_height, k_depth]
float32[] plane_point                          #Plane metric point, [i_width, j_height, k_depth]
sensor_msgs/PointCloud2 pcl                    #Point cloud
