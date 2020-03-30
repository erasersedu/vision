///3D FRONT LINE FINDER
#include "erasers_tools/planefinder.h"

//Data structures for changing the point of view

plane3D::plane3D()
{
	debug = false;
	isplane = false;

};
plane3D::~plane3D()
{
};
//******************

cv::Mat getNormals(cv::Mat pointCloud, int delta)
{
  cv::Mat normals = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_32FC3);

  cv::Point3f topLeft;
  cv::Point3f topRight;
  cv::Point3f downLeft;
  cv::Point3f downRight;

  cv::Point3f vec_1;
  cv::Point3f vec_2;
  cv::Point3f normal;

  pcl::PointXYZRGB p_tmp;

  for (int h = 0; h < pointCloud.rows; h++)
	for (int w = 0; w < pointCloud.cols; w++)
	{			
		// Getting Vectors
		topLeft = pointCloud.at<cv::Vec3f>(h-delta, w-delta);
		topRight = pointCloud.at<cv::Vec3f>(h+delta, w-delta);
		downLeft = pointCloud.at<cv::Vec3f>(h-delta, w+delta);
		downRight = pointCloud.at<cv::Vec3f>(h+delta, w+delta);

		if( topLeft.x == 0.0 && topLeft.y == 0.0 && topLeft.z == 0.0 )
			continue; 
		if( topRight.x == 0.0 && topRight.y == 0.0 && topRight.z == 0.0 )
			continue; 
		if( downLeft.x == 0.0 && downLeft.y == 0.0 && downLeft.z == 0.0 )
			continue; 
		if( downRight.x == 0.0 && downRight.y == 0.0 && downRight.z == 0.0 )
			continue; 

		// Normals
		vec_1 = topRight - downLeft;
		vec_2 = topLeft - downRight; 

		// Normal by cross product (v x h)
		normal  = vec_2.cross(vec_1);

		// Make normal unitary and assignin to mat
		float mod = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
		if(mod != 0.0f)
		{
			if( normal.y < 0 )
				normal *= -1; 

			normals.at<cv::Vec3f>(h, w) = ( 1.0f / mod ) * normal;
		}
	}

  return normals;
};

plane3D getHorizontalPlane(cv::Mat pointCloud, cv::Mat mask, double maxDistPointToPlane, int maxIterations, int minPointsForPlane)
{
  // Getting mask indexes
  cv::Point3f pcaPoint, pcaNormal;
  std::vector< cv::Point2i > indexes, indices;

  for( int i=0; i<mask.rows; i++)
	for(int j=0; j< mask.cols; j++) 
	{
		if( mask.at<uchar>(i,j) != 0 )
			indexes.push_back( cv::Point(j,i) ); 
	}

  int counter = 0;
  int planeSize = 0;
  while( counter++ < maxIterations && indexes.size() > minPointsForPlane)
  {
	cv::Point3f p1  = pointCloud.at< cv::Vec3f >( indexes[rand()%indexes.size()] ); 
	cv::Point3f p2  = pointCloud.at< cv::Vec3f >( indexes[rand()%indexes.size()] ); 
	cv::Point3f p3  = pointCloud.at< cv::Vec3f >( indexes[rand()%indexes.size()] ); 

	//Get plane equation from points
	cv::Point3f p12 = p2 - p1; 
	cv::Point3f p13 = p3 - p1; 

	cv::Point3f normal = p12.cross( p13 ); 

	if( normal == cv::Point3f(0,0,0) )
		continue; 

	normal *= 1 / cv::norm( normal ); 

	double a = normal.x;
	double b = normal.y;
	double c = normal.z;
	double d = - ( normal.x*p1.x + normal.y*p1.y + normal.z*p1.z );

	if( std::abs(b < 0.99) )
		continue; 

	// For all points, check distance to candidate plane (getting inliers) 
	std::vector< cv::Point3f > inliers; 
	double num, den, dist;
	for( int i=0; i < (int)indexes.size(); i++)
	{
		cv::Point3f p = pointCloud.at<cv::Vec3f>(indexes[i]);

		num = a*p.x + b*p.y + c*p.z + d ;
		den = std::sqrt( a*a + b*b + c*c ); 

		dist = fabs(num/den);

		if( dist < maxDistPointToPlane )
			inliers.push_back(p); 
	}

	if( inliers.size() < minPointsForPlane || inliers.size() < planeSize )
		continue; 

	// Getting plane using PCA
	cv::PCA pca1( cv::Mat(inliers).reshape(1), cv::Mat(), CV_PCA_DATA_AS_ROW); 
	cv::Point3f pcaNormal1( pca1.eigenvectors.at<float>(2,0), pca1.eigenvectors.at<float>(2,1), pca1.eigenvectors.at<float>(2,2) ); 
	cv::Point3f pcaPoint1(pca1.mean.at<float>(0,0), pca1.mean.at<float>(0,1), pca1.mean.at<float>(0,2));

	//Get plane equation from normal and point
	if( pcaNormal != cv::Point3f(0,0,0) )
	{
		pcaNormal1 *= 1 / cv::norm( pcaNormal1 );

		a = pcaNormal1.x;
		b = pcaNormal1.y;
		c = pcaNormal1.z;
		d = - ( pcaNormal1.x*pcaPoint1.x + pcaNormal1.y*pcaPoint1.y + pcaNormal1.z*pcaPoint1.z );
	}

	// Obtaining Refined Plane
	std::vector< cv::Point3f > planePoints;
	std::vector< cv::Point2i > planeIndexes;

	for( int i=0; i<(int)indexes.size() ; i++)
	{
		cv::Point3f p = pointCloud.at<cv::Vec3f>(indexes[i]);

		num = a*p.x + b*p.y + c*p.z + d ;
		den = std::sqrt( a*a + b*b + c*c ); 

		dist = fabs(num/den);

		if( dist < maxDistPointToPlane )
		{
			planePoints.push_back( p );
			planeIndexes.push_back( indexes[i] ); 
		}
	}

	//indexes = planeIndexes; 

        if (planeIndexes.size() > planeSize)
	{
		// GETTING REFINED PLANES 
		cv::PCA pca2( cv::Mat(planePoints).reshape(1), cv::Mat(), CV_PCA_DATA_AS_ROW); 
		pcaNormal = cv::Point3f( pca2.eigenvectors.at<float>(2,0), pca2.eigenvectors.at<float>(2,1), pca2.eigenvectors.at<float>(2,2) ); 
		pcaPoint = cv::Point3f( pca2.mean.at<float>(0,0), pca2.mean.at<float>(0,1), pca2.mean.at<float>(0,2));

		planeSize = planeIndexes.size();
		indices = planeIndexes;
	}
	
  }

  //Points in plane
  cv::Mat plane = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC1);
  for (int i = 0; i < indices.size(); i++)
  {
	 plane.at<uchar>(indices[i]) = (unsigned char)255;
  }

  int kernel_elem = 2; //0: Rect - 1: Cross - 2: Ellipse
  int morph_size = 2; //kernel_size = 2*morph_size +1
  cv::Mat element = cv::getStructuringElement( kernel_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );

  cv::morphologyEx( plane, plane, cv::MORPH_CLOSE, element );
  //cv::erode( plane, plane, element );

  //Get biggest contour
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours( plane, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE );

  double area = 0; int idx = 0;
  for( int i = 0; i < contours.size(); i++ )
     {
	if ( cv::contourArea(contours[i]) > area )
	{
		idx = i;
		area = cv::contourArea(contours[i]);
	}
     }

  std::vector< cv::Point2f > planeXZ;
  cv::Mat bigplane = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC1);
  for( int i=0; i<(int)indices.size() ; i++)
  {
	if( cv::pointPolygonTest(contours[idx], indices[i], false) > 0)
	{
		cv::Point3f p = pointCloud.at<cv::Vec3f>(indices[i]);
		bigplane.at<uchar>(indices[i]) = (unsigned char)255;
		planeXZ.push_back( cv::Point2f(p.x, p.z) );
	}
  }

  cv::morphologyEx( bigplane, bigplane, cv::MORPH_CLOSE, element );

  std::vector< cv::Point2f > convexHullXZ;

  //Generate response
  plane3D _plane;

  try 
  {
      cv::convexHull(planeXZ, convexHullXZ);
  } catch (const std::exception &e)
  {
  	std::cout << "plane_finder.no_plane" << std::endl;
	return _plane;
  }

  _plane.isplane = true;
  _plane.pointCloud = pointCloud.clone();

  _plane.point = pcaPoint;
  _plane.normal = pcaNormal;

  _plane.plane = plane.clone();
  _plane.indices = indices;

  _plane.bigplane = bigplane.clone();
  _plane.contour = contours[idx];
  _plane.convexHullXZ = convexHullXZ;

  return _plane; 
};

plane3D getVerticalPlane(cv::Mat pointCloud, cv::Mat mask, double maxDistPointToPlane, int maxIterations, int minPointsForPlane)
{
  // Getting mask indexes
  cv::Point3f pcaPoint, pcaNormal;
  std::vector< cv::Point2i > indexes, indices;

  for( int i=0; i<mask.rows; i++)
	for(int j=0; j< mask.cols; j++) 
	{
		if( mask.at<uchar>(i,j) != 0 )
			indexes.push_back( cv::Point(j,i) ); 
	}

  int counter = 0;
  int planeSize = 0;
  while( counter++ < maxIterations && indexes.size() > minPointsForPlane)
  {
	cv::Point3f p1  = pointCloud.at< cv::Vec3f >( indexes[rand()%indexes.size()] ); 
	cv::Point3f p2  = pointCloud.at< cv::Vec3f >( indexes[rand()%indexes.size()] ); 
	cv::Point3f p3  = pointCloud.at< cv::Vec3f >( indexes[rand()%indexes.size()] ); 

	//Get plane equation from points
	cv::Point3f p12 = p2 - p1; 
	cv::Point3f p13 = p3 - p1; 

	cv::Point3f normal = p12.cross( p13 ); 

	if( normal == cv::Point3f(0,0,0) )
		continue; 

	normal *= 1 / cv::norm( normal ); 

	double a = normal.x;
	double b = normal.y;
	double c = normal.z;
	double d = - ( normal.x*p1.x + normal.y*p1.y + normal.z*p1.z );

	if( std::abs(b > 0.01) )
		continue; 

	// For all points, check distance to candidate plane (getting inliers) 
	std::vector< cv::Point3f > inliers; 
	double num, den, dist;
	for( int i=0; i < (int)indexes.size(); i++)
	{
		cv::Point3f p = pointCloud.at<cv::Vec3f>(indexes[i]);

		num = a*p.x + b*p.y + c*p.z + d ;
		den = std::sqrt( a*a + b*b + c*c ); 

		dist = fabs(num/den);

		if( dist < maxDistPointToPlane )
			inliers.push_back(p); 
	}

	if( inliers.size() < minPointsForPlane || inliers.size() < planeSize )
		continue; 

	// Getting plane using PCA
	cv::PCA pca1( cv::Mat(inliers).reshape(1), cv::Mat(), CV_PCA_DATA_AS_ROW); 
	cv::Point3f pcaNormal1( pca1.eigenvectors.at<float>(2,0), pca1.eigenvectors.at<float>(2,1), pca1.eigenvectors.at<float>(2,2) ); 
	cv::Point3f pcaPoint1(pca1.mean.at<float>(0,0), pca1.mean.at<float>(0,1), pca1.mean.at<float>(0,2));

	//Get plane equation from normal and point
	if( pcaNormal != cv::Point3f(0,0,0) )
	{
		pcaNormal1 *= 1 / cv::norm( pcaNormal1 );

		a = pcaNormal1.x;
		b = pcaNormal1.y;
		c = pcaNormal1.z;
		d = - ( pcaNormal1.x*pcaPoint1.x + pcaNormal1.y*pcaPoint1.y + pcaNormal1.z*pcaPoint1.z );
	}

	// Obtaining Refined Plane
	std::vector< cv::Point3f > planePoints;
	std::vector< cv::Point2i > planeIndexes;

	for( int i=0; i<(int)indexes.size() ; i++)
	{
		cv::Point3f p = pointCloud.at<cv::Vec3f>(indexes[i]);

		num = a*p.x + b*p.y + c*p.z + d ;
		den = std::sqrt( a*a + b*b + c*c ); 

		dist = fabs(num/den);

		if( dist < maxDistPointToPlane )
		{
			planePoints.push_back( p );
			planeIndexes.push_back( indexes[i] ); 
		}
	}

	//indexes = planeIndexes; 

        if (planeIndexes.size() > planeSize)
	{
		// GETTING REFINED PLANES 
		cv::PCA pca2( cv::Mat(planePoints).reshape(1), cv::Mat(), CV_PCA_DATA_AS_ROW); 
		pcaNormal = cv::Point3f( pca2.eigenvectors.at<float>(2,0), pca2.eigenvectors.at<float>(2,1), pca2.eigenvectors.at<float>(2,2) ); 
		pcaPoint = cv::Point3f( pca2.mean.at<float>(0,0), pca2.mean.at<float>(0,1), pca2.mean.at<float>(0,2));

		planeSize = planeIndexes.size();
		indices = planeIndexes;
	}
	
  }

  //Points in plane
  cv::Mat plane = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC1);
  for (int i = 0; i < indices.size(); i++)
  {
	 plane.at<uchar>(indices[i]) = (unsigned char)255;
  }

  int kernel_elem = 0; //0: Rect - 1: Cross - 2: Ellipse
  int morph_size = 2; //kernel_size = 2*morph_size +1
  cv::Mat element = cv::getStructuringElement( kernel_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );

  cv::morphologyEx( plane, plane, cv::MORPH_CLOSE, element );
  //cv::erode( plane, plane, element );

  //Get biggest contour
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours( plane, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE );

  double area = 0; int idx = 0;
  for( int i = 0; i < contours.size(); i++ )
     {
	if ( cv::contourArea(contours[i]) > area )
	{
		idx = i;
		area = cv::contourArea(contours[i]);
	}
     }

  std::vector< cv::Point2f > planeXY;
  cv::Mat bigplane = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC1);
  for( int i=0; i<(int)indices.size() ; i++)
  {
	if( cv::pointPolygonTest(contours[idx], indices[i], false) > 0)
	{
		cv::Point3f p = pointCloud.at<cv::Vec3f>(indices[i]);
		bigplane.at<uchar>(indices[i]) = (unsigned char)255;
		planeXY.push_back( cv::Point2f(p.x, p.y) );
	}
  }

  cv::morphologyEx( bigplane, bigplane, cv::MORPH_CLOSE, element );

  std::vector< cv::Point2f > convexHullXY;

  //Generate response
  plane3D _plane;

  try 
  {
      cv::convexHull(planeXY, convexHullXY);
  } catch (const std::exception &e)
  {
  	std::cout << "plane_finder.no_plane" << std::endl;
	return _plane;
  }

  _plane.isplane = true;
  _plane.pointCloud = pointCloud.clone();

  _plane.point = pcaPoint;
  _plane.normal = pcaNormal;

  _plane.plane = plane.clone();
  _plane.indices = indices;

  _plane.bigplane = bigplane.clone();
  _plane.contour = contours[idx];
  _plane.convexHullXY = convexHullXY;

  return _plane; 
};
