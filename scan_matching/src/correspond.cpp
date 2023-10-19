#include "scan_matching_skeleton/correspond.h"
#include "cmath"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

// PPICP
void getNaiveCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                            vector<vector<int>> &jump_table, vector<Correspondence> &c, float prob) {

    c.clear();
    int last_best = -1;
    const int n = trans_points.size();
    const int m = old_points.size();
    int min_index = 0;
    int second_min_index = 0;

    //Do for each transformed point point
    for (int ind_trans = 0; ind_trans < n; ++ind_trans) {
        float min_dist = 100000.00;
        // find closest correspondance, mininum distance
        for (int ind_old = 0; ind_old < m; ++ind_old) {
            // get the distance
            float dist = old_points[ind_trans].distToPoint2(&trans_points[ind_old]);
            if (dist < min_dist) { // if distance is smaller
                // update distance and index variables
                min_dist = dist;
                min_index = ind_old;


                if (ind_old == 0) { // if first point
                    second_min_index = ind_old + 1; // update min index with 2nd point
                } else {
                    second_min_index = ind_old - 1; // update min index with prev point
                }
            }
        }
        c.push_back(Correspondence(&trans_points[ind_trans], &points[ind_trans], &old_points[min_index], &old_points[second_min_index]));
    }
}

// PLICP
void getCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                       vector<vector<int>> &jump_table, vector<Correspondence> &c, float prob) {

    // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
    // use helper functions and structs in transform.h and correspond.h
    // input : old_points : vector of struct points containing the old points (points of the previous frame)
    // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
    // input : points : vector of struct points containing the new points
    // input : jump_table : jump table computed using the helper functions from the transformed and old points
    // input : c: vector of struct correspondences . This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
    // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point.
    //Initializecorrespondences
    c.clear();
    int last_best = -1; // Out of the main loop, we remember the last match found.
    const int trans_size = trans_points.size();
    const int old_size = old_points.size();

    //Do for each point
    for (int ind_trans = 0; ind_trans < min(old_size, trans_size); ++ind_trans) {
        // Current best match, and its distance
        int best = 0;
        double best_dist = numeric_limits<double>::infinity(); // infinity value

        int second_best = 0;

        // index in current loop
        int start_index = ind_trans;

        // If last match was succesful, then start at that index + 1
        int we_start_at = (last_best != -1) ? (last_best + 1) : start_index;


        // Search is conducted in two directions: up and down
        int up = we_start_at+1;
        int down = we_start_at;

        // Distance of last point examined in the up (down) direction.
        double last_dist_up=100000-1;
        double last_dist_down=100000; // offset for first case pass

        // True if search is finished in the up (down) direction
        bool up_stopped = false;
        bool down_stopped = false;

        // Until the search is stopped in both directions...
        while (!(up_stopped && down_stopped)) {
            // Should we try to explore up or down?
            bool now_up = !up_stopped && (last_dist_up < last_dist_down);

            // Now two symmetric chunks of code, the now_up and the !now_up
            if (now_up) {
                // If we have finished the points to search, we stop.
                if (up >= old_size) {
                    up_stopped = true;
                    continue;
                }

                // calculate distance between new points and last points at X and Y using distance formula
                last_dist_up = pow(trans_points[ind_trans].getX()-old_points[up].getX(), 2)
                               + pow(trans_points[ind_trans].getY()-old_points[up].getY(), 2);

                // If it is less than the best point, up is our best guess so far
                if ((last_dist_up < best_dist)) {
                    best=up;
                    best_dist = last_dist_up;
                }

                /** If we are moving away from start_cell we can compute a bound
                 *  for early stopping. Currently our best point has distance
                 *  best_dist; we can compute the minimum distance to point for
                 *  points j > up */
                if (up > start_index) {
                    double best_point = (up-ind_trans);
                    double min_dist_up = sin(best_point)*trans_points[ind_trans].r;

                    if (pow(min_dist_up, 2) > best_dist) {
                        /** If going up we can’t make better than best_dist,
                            then we stop searching in the "up" direction */
                        up_stopped = true;
                        continue;
                    }

                    // If we are moving away, then we can implement the jump tables optimization.
                    up = (old_points[up].r < trans_points[ind_trans].r) ? jump_table[up][1] : jump_table[up][0];
                } else {
                    /** If we are moving towards "start_cell", we can't do any of the 
                     *  previous optimization and we just move to the next point. */
                    up++;
                }

            } else { // This is the specular part of the previous chunk of code
                if (down <= -1) {
                    down_stopped = true;
                    continue;
                }

                last_dist_down = pow(trans_points[ind_trans].getX()-old_points[down].getX(), 2)
                                 + pow(trans_points[ind_trans].getY()-old_points[down].getY(), 2);

                if ((last_dist_down < best_dist)) {
                    best = down;
                    best_dist = last_dist_down;
                }

                if(down < start_index) {
                    double best_point = (down-ind_trans);
                    double min_dist_down = sin(best_point)*trans_points[ind_trans].r;

                    if(pow(min_dist_down, 2) > best_dist) {
                        down_stopped = true;
                        continue;
                    }

                    // compare magnitudes of prev jump dist and transformed points jump dist
                    down = (old_points[down].r < trans_points[ind_trans].r) ? jump_table[down][3] : jump_table[down][2];
                } else {
                    down--;
                }
            }
        }

        // For the next point, we will start at best
        last_best = best;

        // if our best is out of our bounds, give a proper second_best.
        if (best<=0) {
            second_best=best+1;
        } else {
            second_best=best-1;
        }

        // Push the correspondence found to c.
        c.push_back(Correspondence(&trans_points[ind_trans], &points[ind_trans], &old_points[best], &old_points[second_best]));
    }
}

void computeJump(vector<vector<int>> &table, vector<Point> &points) {
    table.clear();
    int n = points.size();
    for (int i = 0; i < n; ++i) {
        vector<int> v = {n, n, -1, -1};
        for (int j = i + 1; j < n; ++j) {
            if (points[j].r < points[i].r) {
                v[UP_SMALL] = j;
                break;
            }
        }
        for (int j = i + 1; j < n; ++j) {
            if (points[j].r > points[i].r) {
                v[UP_BIG] = j;
                break;
            }
        }
        for (int j = i - 1; j >= 0; --j) {
            if (points[j].r < points[i].r) {
                v[DOWN_SMALL] = j;
                break;
            }
        }
        for (int j = i - 1; j >= 0; --j) {
            if (points[j].r > points[i].r) {
                v[DOWN_BIG] = j;
                break;
            }
        }
        table.push_back(v);
    }
}
