/*
 *  Harversine.c
 *  Haversine
 *
 *  Created by Jaime Rios on 2/16/08.
 *
 */

#include "haversine.h"

/**********************************************************************
 Haversine Formula
 R = earth’s radius (mean radius = 6,371km)
 Δlat = lat2− lat1
 Δlong = long2− long1
 a = sin²(Δlat/2) + cos(lat1).cos(lat2).sin²(Δlong/2)
 c = 2.atan2(√a, √(1−a))
 d = R.c

 JavaScript Example from http://www.movable-type.co.uk/scripts/latlong.html
 var R = 6371; // km
 var dLat = (lat2-lat1).toRad();
 var dLon = (lon2-lon1).toRad();
 lat1 = lat1.toRad(), lat2 = lat2.toRad();

 var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                 Math.cos(lat1.toRad()) * Math.cos(lat2.toRad()) *
                 Math.sin(dLon/2) * Math.sin(dLon/2);
 var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
 var d = R * c;
 */
double calculate_distance(const double latitude1,
                        const double longitude1,
                        const double latitude2,
                        const double longitude2)
{
  const double earths_radius = 6371;

  // Get the difference between our two points then convert the difference into radians
  const auto lat_delta = convert(latitude2 - latitude1);
  const auto lon_delta = convert(longitude2 - longitude1);

  const double converted_lat1 = convert(latitude1);
  const double converted_lat2 = convert(latitude2);

  const auto a =
      pow(sin(lat_delta / 2), 2) + cos(converted_lat1) * cos(converted_lat2) * pow(sin(lon_delta / 2), 2);

  const auto c = 2 * atan2(sqrt(a), sqrt(1 - a));
  const auto d = earths_radius * c;

  return d;
}

// convert our passed value to radians_t
double convert(const double angle)
{
  return angle * (M_PI / 180);
}
