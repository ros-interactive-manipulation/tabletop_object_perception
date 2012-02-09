/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#include <distance_field/propagation_distance_field.h>
#include <visualization_msgs/Marker.h>

namespace distance_field
{

PropagationDistanceField::~PropagationDistanceField()
{
}

PropagationDistanceField::PropagationDistanceField(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z, double max_distance):
      DistanceField<PropDistanceFieldVoxel>(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, PropDistanceFieldVoxel(max_distance))
{
  max_distance_ = max_distance;
  int max_dist_int = ceil(max_distance_/resolution);
  max_distance_sq_ = (max_dist_int*max_dist_int);
  initNeighborhoods();

  // create a sqrt table:
  sqrt_table_.resize(max_distance_sq_+1);
  for (int i=0; i<=max_distance_sq_; ++i)
    sqrt_table_[i] = sqrt(double(i))*resolution;
}

int PropagationDistanceField::eucDistSq(int3 point1, int3 point2)
{
  int dx = point1.x() - point2.x();
  int dy = point1.y() - point2.y();
  int dz = point1.z() - point2.z();
  return dx*dx + dy*dy + dz*dz;
}


void PropagationDistanceField::updatePointsInField(const std::vector<tf::Vector3>& points)
{
  std::vector<int3> points_added;
  std::vector<int3> points_removed;

  // Compare and figure out what points are new,
  // and what points are to be deleted
  points_added.reserve(points.size());
  for( unsigned int i=0; i<points.size(); i++)
  {
    // Convert to voxel coordinates
    const int3 loc(points[i].x(), points[i].y(), points[i].z());
    const PropDistanceFieldVoxel& voxel = getCell( loc.x(), loc.y(), loc.z() );
    if( voxel.distance_square_ != 0 )
    {
      points_added.push_back( loc );
    }
  }

  // TODO - calculate the points removed

  if( points_removed.size() > 0 )
  {
    reset();
    //addNewObstacleVoxels(points);	//FIXME
  }
  else
  {
    addNewObstacleVoxels(points_added);
    addNewObstacleVoxels( points_added );
  }
}

void PropagationDistanceField::addPointsToField(const std::vector<tf::Vector3>& points)
{
  std::vector<int3> points_added;

  points_added.reserve(points.size());
  for( unsigned int i=0; i<points.size(); i++)
  {
    // Convert to voxel coordinates
    int3 voxel_loc;
    bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(),
                              voxel_loc.x(), voxel_loc.y(), voxel_loc.z() );

    if( valid )
    {
      const PropDistanceFieldVoxel& voxel = getCell( voxel_loc.x(), voxel_loc.y(), voxel_loc.z() );

      if( voxel.distance_square_ != 0 )
      {
        // Add point if it's within the grid and not already an object voxel
        points_added.push_back( voxel_loc );
      }
    }
  }

  addNewObstacleVoxels( points_added );
}

void PropagationDistanceField::addNewObstacleVoxels(const std::vector<int3>& voxels)
{
  // initialize the bucket queue
  bucket_queue_.resize(max_distance_sq_+1);

  bucket_queue_[0].reserve(voxels.size());
  // first mark all the voxels as distance=0, and add them to the queue
  int x, y, z, nx, ny, nz;
  int3 loc;
  int initial_update_direction = getDirectionNumber(0,0,0);
  for (unsigned int i=0; i<voxels.size(); ++i)
  {
    x = voxels[i].x();
    y = voxels[i].y();
    z = voxels[i].z();
    bool valid = isCellValid( x, y, z);
    if (!valid)
      continue;
    PropDistanceFieldVoxel& voxel = getCell(x,y,z);
    voxel.distance_square_ = 0;
    voxel.closest_point_ = voxels[i];
    voxel.location_ = voxels[i];
    voxel.update_direction_ = initial_update_direction;
    bucket_queue_[0].push_back(&voxel);
  }

  // now process the queue:
  for (unsigned int i=0; i<bucket_queue_.size(); ++i)
  {
    std::vector<PropDistanceFieldVoxel*>::iterator list_it = bucket_queue_[i].begin();
    while(list_it!=bucket_queue_[i].end())
    {
      PropDistanceFieldVoxel* vptr = *list_it;

      x = vptr->location_.x();
      y = vptr->location_.y();
      z = vptr->location_.z();

      // select the neighborhood list based on the update direction:
      std::vector<int3 >* neighborhood;
      int D = i;
      if (D>1)
        D=1;
      // avoid a possible segfault situation:
      if (vptr->update_direction_<0 || vptr->update_direction_>26)
      {
   //     ROS_WARN("Invalid update direction detected: %d", vptr->update_direction_);
        ++list_it;
        continue;
      }

      neighborhood = &neighborhoods_[D][vptr->update_direction_];

      for (unsigned int n=0; n<neighborhood->size(); n++)
      {
        int dx = (*neighborhood)[n].x();
        int dy = (*neighborhood)[n].y();
        int dz = (*neighborhood)[n].z();
        nx = x + dx;
        ny = y + dy;
        nz = z + dz;
        if (!isCellValid(nx,ny,nz))
          continue;

        // the real update code:
        // calculate the neighbor's new distance based on my closest filled voxel:
        PropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);
        loc.x() = nx;
        loc.y() = ny;
        loc.z() = nz;
        int new_distance_sq = eucDistSq(vptr->closest_point_, loc);
        if (new_distance_sq > max_distance_sq_)
          continue;
        if (new_distance_sq < neighbor->distance_square_)
        {
          // update the neighboring voxel
          neighbor->distance_square_ = new_distance_sq;
          neighbor->closest_point_ = vptr->closest_point_;
          neighbor->location_ = loc;
          neighbor->update_direction_ = getDirectionNumber(dx, dy, dz);

          // and put it in the queue:
          bucket_queue_[new_distance_sq].push_back(neighbor);
        }
      }

      ++list_it;
    }
    bucket_queue_[i].clear();
  }

}

void PropagationDistanceField::reset()
{
  VoxelGrid<PropDistanceFieldVoxel>::reset(PropDistanceFieldVoxel(max_distance_sq_));
}

void PropagationDistanceField::initNeighborhoods()
{
  // first initialize the direction number mapping:
  direction_number_to_direction_.resize(27);
  for (int dx=-1; dx<=1; ++dx)
  {
    for (int dy=-1; dy<=1; ++dy)
    {
      for (int dz=-1; dz<=1; ++dz)
      {
        int direction_number = getDirectionNumber(dx, dy, dz);
        int3 n_point( dx, dy, dz);
        direction_number_to_direction_[direction_number] = n_point;
      }
    }
  }

  neighborhoods_.resize(2);
  for (int n=0; n<2; n++)
  {
    neighborhoods_[n].resize(27);
    // source directions
    for (int dx=-1; dx<=1; ++dx)
    {
      for (int dy=-1; dy<=1; ++dy)
      {
        for (int dz=-1; dz<=1; ++dz)
        {
          int direction_number = getDirectionNumber(dx, dy, dz);
          // target directions:
          for (int tdx=-1; tdx<=1; ++tdx)
          {
            for (int tdy=-1; tdy<=1; ++tdy)
            {
              for (int tdz=-1; tdz<=1; ++tdz)
              {
                if (tdx==0 && tdy==0 && tdz==0)
                  continue;
                if (n>=1)
                {
                  if ((abs(tdx) + abs(tdy) + abs(tdz))!=1)
                    continue;
                  if (dx*tdx<0 || dy*tdy<0 || dz*tdz <0)
                    continue;
                }
                int3 n_point(tdx,tdy,tdz);
                neighborhoods_[n][direction_number].push_back(n_point);
              }
            }
          }
          //printf("n=%d, dx=%d, dy=%d, dz=%d, neighbors = %d\n", n, dx, dy, dz, neighborhoods_[n][direction_number].size());
        }
      }
    }
  }

}

int PropagationDistanceField::getDirectionNumber(int dx, int dy, int dz) const
{
  return (dx+1)*9 + (dy+1)*3 + dz+1;
}

SignedPropagationDistanceField::~SignedPropagationDistanceField()
{
}

SignedPropagationDistanceField::SignedPropagationDistanceField(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z, double max_distance):
      DistanceField<SignedPropDistanceFieldVoxel>(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, SignedPropDistanceFieldVoxel(max_distance,0))
{
  max_distance_ = max_distance;
  int max_dist_int = ceil(max_distance_/resolution);
  max_distance_sq_ = (max_dist_int*max_dist_int);
  initNeighborhoods();

  // create a sqrt table:
  sqrt_table_.resize(max_distance_sq_+1);
  for (int i=0; i<=max_distance_sq_; ++i)
    sqrt_table_[i] = sqrt(double(i))*resolution;
}

int SignedPropagationDistanceField::eucDistSq(int3 point1, int3 point2)
{
  int dx = point1.x() - point2.x();
  int dy = point1.y() - point2.y();
  int dz = point1.z() - point2.z();
  return dx*dx + dy*dy + dz*dz;
}

void SignedPropagationDistanceField::addPointsToField(const std::vector<tf::Vector3> points)
{
  // initialize the bucket queue
  positive_bucket_queue_.resize(max_distance_sq_+1);
  negative_bucket_queue_.resize(max_distance_sq_+1);

  positive_bucket_queue_[0].reserve(points.size());
  negative_bucket_queue_[0].reserve(points.size());

  for(int x = 0; x < num_cells_[0]; x++)
  {
    for(int y = 0; y < num_cells_[1]; y++)
    {
      for(int z = 0; z < num_cells_[2]; z++)
      {
        SignedPropDistanceFieldVoxel& voxel = getCell(x,y,z);
        voxel.closest_negative_point_.x() = x;
        voxel.closest_negative_point_.y() = y;
        voxel.closest_negative_point_.z() = z;
        voxel.negative_distance_square_ = 0;
      }
    }
  }

  // first mark all the points as distance=0, and add them to the queue
  int x, y, z, nx, ny, nz;
  int3 loc;
  int initial_update_direction = getDirectionNumber(0,0,0);
  for (unsigned int i=0; i<points.size(); ++i)
  {
    bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(), x, y, z);
    if (!valid)
      continue;
    SignedPropDistanceFieldVoxel& voxel = getCell(x,y,z);
    voxel.positive_distance_square_ = 0;
    voxel.negative_distance_square_ = max_distance_sq_;
    voxel.closest_positive_point_.x() = x;
    voxel.closest_positive_point_.y() = y;
    voxel.closest_positive_point_.z() = z;
    voxel.closest_negative_point_.x() = SignedPropDistanceFieldVoxel::UNINITIALIZED;
    voxel.closest_negative_point_.y() = SignedPropDistanceFieldVoxel::UNINITIALIZED;
    voxel.closest_negative_point_.z() = SignedPropDistanceFieldVoxel::UNINITIALIZED;
    voxel.location_.x() = x;
    voxel.location_.y() = y;
    voxel.location_.z() = z;
    voxel.update_direction_ = initial_update_direction;
    positive_bucket_queue_[0].push_back(&voxel);
  }

  // now process the queue:
  for (unsigned int i=0; i<positive_bucket_queue_.size(); ++i)
  {
    std::vector<SignedPropDistanceFieldVoxel*>::iterator list_it = positive_bucket_queue_[i].begin();
    while(list_it!=positive_bucket_queue_[i].end())
    {
      SignedPropDistanceFieldVoxel* vptr = *list_it;

      x = vptr->location_.x();
      y = vptr->location_.y();
      z = vptr->location_.z();

      // select the neighborhood list based on the update direction:
      std::vector<int3 >* neighborhood;
      int D = i;
      if (D>1)
        D=1;
      // avoid a possible segfault situation:
      if (vptr->update_direction_<0 || vptr->update_direction_>26)
      {
   //     ROS_WARN("Invalid update direction detected: %d", vptr->update_direction_);
        ++list_it;
        continue;
      }

      neighborhood = &neighborhoods_[D][vptr->update_direction_];

      for (unsigned int n=0; n<neighborhood->size(); n++)
      {
        int dx = (*neighborhood)[n].x();
        int dy = (*neighborhood)[n].y();
        int dz = (*neighborhood)[n].z();
        nx = x + dx;
        ny = y + dy;
        nz = z + dz;
        if (!isCellValid(nx,ny,nz))
          continue;

        // the real update code:
        // calculate the neighbor's new distance based on my closest filled voxel:
        SignedPropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);
        loc.x() = nx;
        loc.y() = ny;
        loc.z() = nz;
        int new_distance_sq = eucDistSq(vptr->closest_positive_point_, loc);
        if (new_distance_sq > max_distance_sq_)
          continue;
        if (new_distance_sq < neighbor->positive_distance_square_)
        {
          // update the neighboring voxel
          neighbor->positive_distance_square_ = new_distance_sq;
          neighbor->closest_positive_point_ = vptr->closest_positive_point_;
          neighbor->location_ = loc;
          neighbor->update_direction_ = getDirectionNumber(dx, dy, dz);

          // and put it in the queue:
          positive_bucket_queue_[new_distance_sq].push_back(neighbor);
        }
      }

      ++list_it;
    }
    positive_bucket_queue_[i].clear();
  }


  for(unsigned int i = 0; i < points.size(); i++)
    {
      bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(), x, y, z);
      if(!valid)
        continue;

      for(int dx = -1; dx <= 1; dx ++)
      {
        for(int dy = -1; dy<= 1; dy ++)
        {
          for(int dz = -1; dz <= 1; dz++)
          {
            nx = x + dx;
            ny = y + dy;
            nz = z + dz;

            if(!isCellValid(nx, ny, nz))
              continue;

            SignedPropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);

            if(neighbor->closest_negative_point_.x() != SignedPropDistanceFieldVoxel::UNINITIALIZED)
            {
              neighbor->update_direction_ = initial_update_direction;
              negative_bucket_queue_[0].push_back(neighbor);
            }
          }
        }
      }

    }

  for (unsigned int i=0; i<negative_bucket_queue_.size(); ++i)
  {
    std::vector<SignedPropDistanceFieldVoxel*>::iterator list_it = negative_bucket_queue_[i].begin();
    while(list_it!=negative_bucket_queue_[i].end())
    {
      SignedPropDistanceFieldVoxel* vptr = *list_it;

      x = vptr->location_.x();
      y = vptr->location_.y();
      z = vptr->location_.z();

      // select the neighborhood list based on the update direction:
      std::vector<int3 >* neighborhood;
      int D = i;
      if (D>1)
        D=1;
      // avoid a possible segfault situation:
      if (vptr->update_direction_<0 || vptr->update_direction_>26)
      {
   //     ROS_WARN("Invalid update direction detected: %d", vptr->update_direction_);
        ++list_it;
        continue;
      }

      neighborhood = &neighborhoods_[D][vptr->update_direction_];

      for (unsigned int n=0; n<neighborhood->size(); n++)
      {
        int dx = (*neighborhood)[n].x();
        int dy = (*neighborhood)[n].y();
        int dz = (*neighborhood)[n].z();
        nx = x + dx;
        ny = y + dy;
        nz = z + dz;
        if (!isCellValid(nx,ny,nz))
          continue;

        // the real update code:
        // calculate the neighbor's new distance based on my closest filled voxel:
        SignedPropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);
        loc.x() = nx;
        loc.y() = ny;
        loc.z() = nz;
        int new_distance_sq = eucDistSq(vptr->closest_negative_point_, loc);
        if (new_distance_sq > max_distance_sq_)
          continue;
        if (new_distance_sq < neighbor->negative_distance_square_)
        {
          // update the neighboring voxel
          neighbor->negative_distance_square_ = new_distance_sq;
          neighbor->closest_negative_point_ = vptr->closest_negative_point_;
          neighbor->location_ = loc;
          neighbor->update_direction_ = getDirectionNumber(dx, dy, dz);

          // and put it in the queue:
          negative_bucket_queue_[new_distance_sq].push_back(neighbor);
        }
      }

      ++list_it;
    }
    negative_bucket_queue_[i].clear();
  }

}

void SignedPropagationDistanceField::reset()
{
  VoxelGrid<SignedPropDistanceFieldVoxel>::reset(SignedPropDistanceFieldVoxel(max_distance_sq_, 0));
}

void SignedPropagationDistanceField::initNeighborhoods()
{
  // first initialize the direction number mapping:
  direction_number_to_direction_.resize(27);
  for (int dx=-1; dx<=1; ++dx)
  {
    for (int dy=-1; dy<=1; ++dy)
    {
      for (int dz=-1; dz<=1; ++dz)
      {
        int direction_number = getDirectionNumber(dx, dy, dz);
        int3 n_point( dx, dy, dz);
        direction_number_to_direction_[direction_number] = n_point;
      }
    }
  }

  neighborhoods_.resize(2);
  for (int n=0; n<2; n++)
  {
    neighborhoods_[n].resize(27);
    // source directions
    for (int dx=-1; dx<=1; ++dx)
    {
      for (int dy=-1; dy<=1; ++dy)
      {
        for (int dz=-1; dz<=1; ++dz)
        {
          int direction_number = getDirectionNumber(dx, dy, dz);
          // target directions:
          for (int tdx=-1; tdx<=1; ++tdx)
          {
            for (int tdy=-1; tdy<=1; ++tdy)
            {
              for (int tdz=-1; tdz<=1; ++tdz)
              {
                if (tdx==0 && tdy==0 && tdz==0)
                  continue;
                if (n>=1)
                {
                  if ((abs(tdx) + abs(tdy) + abs(tdz))!=1)
                    continue;
                  if (dx*tdx<0 || dy*tdy<0 || dz*tdz <0)
                    continue;
                }
                int3 n_point(tdx,tdy,tdz);
                neighborhoods_[n][direction_number].push_back(n_point);
              }
            }
          }
          //printf("n=%d, dx=%d, dy=%d, dz=%d, neighbors = %d\n", n, dx, dy, dz, neighborhoods_[n][direction_number].size());
        }
      }
    }
  }



}

int SignedPropagationDistanceField::getDirectionNumber(int dx, int dy, int dz) const
{
  return (dx+1)*9 + (dy+1)*3 + dz+1;
}


}
