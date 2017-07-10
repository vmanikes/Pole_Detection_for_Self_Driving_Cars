import pcl
import numpy as np
import math
from geo2cart import geo2cart

def filterOutliers(point_cloud,mean,sd):
    outlier_filter = point_cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean)
    outlier_filter.set_std_dev_mul_thresh(sd)
    outlier_filter.set_negative(False)
    Neg_outliers = outlier_filter.filter()
    filter_file = open('Filter', 'w')
    for point in Neg_outliers:
        line = str(point[0]) + " " + str(point[1]) + " " + str(point[2])
        filter_file.write(line)
        filter_file.write("\n")
    filter_file.close()

    return Neg_outliers

def clear_objects(Neg_outliers,index,thresh):
    rem_elements = Neg_outliers.make_kdtree_flann()
    k_indices, k_sqr_distances = rem_elements.nearest_k_search_for_cloud(Neg_outliers, index)
    distances = np.sum(k_sqr_distances, axis=1)
    a = 1500000
    blocks = []
    for i in range(np.shape(distances)[0]):
        blocks.extend(k_indices[i]) if distances[i] < float(thresh) else a
    unique_indices = list(set(blocks))
    Neg_outliers = Neg_outliers.extract(unique_indices, negative=True)
    refined = open('Refined', 'w')
    for point in Neg_outliers:
        line = str(point[0]) + " " + str(point[1]) + " " + str(point[2])
        refined.write(line)
        refined.write("\n")
    refined.close()
    return Neg_outliers

def segmentation(Neg_outliers,model,iter):
    segmentation_set = Neg_outliers.make_segmenter_normals(ksearch=50)
    segmentation_set.set_optimize_coefficients(True)
    segmentation_set.set_normal_distance_weight(0.1)
    segmentation_set.set_method_type(pcl.SAC_RANSAC)
    segmentation_set.set_max_iterations(iter)

    if model == pcl.SACMODEL_CYLINDER:
        segmentation_set.set_model_type(model)
        segmentation_set.set_distance_threshold(20)
        segmentation_set.set_radius_limits(0, 10)
        segmented_indices, model = segmentation_set.segment()
        Cylindrical_seg = Neg_outliers.extract(segmented_indices, negative=False)
        print("Generated Cylindrical segments")
        final = open('Cylinder', 'w')
        for point in Cylindrical_seg:
            line = str(point[0]) + " " + str(point[1]) + " " + str(point[2])
            final.write(line)
            final.write("\n")
        final.close()

        return Cylindrical_seg
    else:
        segmentation_set.set_model_type(model)
        segmentation_set.set_distance_threshold(85)
        segment_indices, model = segmentation_set.segment()
        filtered_cloud = Neg_outliers.extract(segment_indices, negative=True)
        filter = filtered_cloud.make_passthrough_filter()
        filter.set_filter_field_name("x")
        filter.set_filter_limits(0, max(latitude))
        filtered_cloud = filter.filter()
        return filtered_cloud

file = open('final_project_point_cloud.fuse', 'r')

print("Detection started")
latitude = []
longitude = []
altitude = []
intensity = []

combined_points = []

index = 1000
thresh = 5000
for line in file:
    temp = []
    k = line.split()
    latitude.append(float(k[0]))
    longitude.append(float(k[1]))
    altitude.append(float(k[2]))
    intensity.append(float(k[3]))

    coord = geo2cart.cartesian(float(k[0]),float(k[1]),float(k[2]))
    combined_points.append([coord[0],coord[1],coord[2]])


combined_points = np.array(combined_points, dtype=np.float32)
point_cloud = pcl.PointCloud()
point_cloud.from_array(combined_points)

Neg_outliers = filterOutliers(point_cloud,mean=50,sd=5)
Neg_outliers = clear_objects(Neg_outliers,index,thresh)

Cylindrical_seg = segmentation(Neg_outliers,model = pcl.SACMODEL_CYLINDER,iter = 1000)
filter_cloud = segmentation(Cylindrical_seg,model = pcl.SACMODEL_CYLINDER,iter = 100)
final = open('Detected_Poles', 'w')
for point in filter_cloud:
    line = str(point[0]) + " " + str(point[1]) + " "+ str(point[2])
    final.write(line)
    final.write("\n")

Print("Detection Completed")

