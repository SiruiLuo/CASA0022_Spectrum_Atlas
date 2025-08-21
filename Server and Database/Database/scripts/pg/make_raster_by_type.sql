
DROP TABLE IF EXISTS rssi_raster_type;

WITH room AS (
  SELECT poly,
         ST_XMin(poly) AS xmin,
         ST_YMin(poly) AS ymin,
         ST_XMax(poly) AS xmax,
         ST_YMax(poly) AS ymax
  FROM room_shape LIMIT 1
),
params AS (
  SELECT (xmax - xmin)/0.10 AS wid_f,
         (ymax - ymin)/0.10 AS hgt_f,
         xmin - 0.10        AS ulx,
         ymax + 0.10        AS uly
  FROM room
),
grid AS (
  SELECT ST_AddBand(
           ST_MakeEmptyRaster(
             CEIL(wid_f)::int + 2,
             CEIL(hgt_f)::int + 2,
             ulx, uly,
             0.10, -0.10, 0, 0, 3857
           ),
           1, '32BF', -9999
         ) AS rast
  FROM params
),
burned AS (
  SELECT ST_Union(
           ST_SetValue(g.rast, 1, s.location, s.rssi)
         ) AS rast
  FROM grid g
  JOIN signal_samples s
    ON s.signal_type = :stype
   AND s.session_id  = :sess
),

smooth AS (
  SELECT ST_MapAlgebraFct(
           rast,                                    
           1,                                       
           'public.mean9vals(double precision[])'::regprocedure
         ) AS rast
  FROM burned
),


clipped AS (
  SELECT ST_Clip(rast, room.poly, TRUE) AS rast
  FROM smooth, room
)
SELECT rast INTO rssi_raster_type FROM clipped;
