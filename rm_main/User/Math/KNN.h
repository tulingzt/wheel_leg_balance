#ifndef __KNN_H
#define __KNN_H

#define M 12    // ÌØÕ÷ÊıÁ¿

typedef struct {
    float distance;
    int label;
} Neighbor;

float calculate_distance(float *data1, float *data2, int length);
void find_nearest_neighbors(Neighbor *neighbors, int k, float distance, int label);
int knn_classify(float *input_data, int k);
float calculate_distance_Manhattan(float *data1, float *data2, int length);


extern float input_data[M];

#endif
