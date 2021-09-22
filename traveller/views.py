import logging

from rest_framework.generics import GenericAPIView
from rest_framework import status
from rest_framework.response import Response

from traveller.utils import DistanceMatrix, RouteFinder

"""
1. For solving Travelling Salesman problem following input is required:
      a. "list_cord": [
                    [75.85959398126745, 22.796033910515693],
                    [75.84514379661131, 22.797202783046455],
                    [75.89372268920312, 22.744358154533295],
                    [75.90357728311815, 22.754718648887533],
                    [75.88515698311818, 22.757859818243723],
                    [75.85584452544431, 22.67783817185593],
                    [75.87114646777299, 22.699688243109538],
                    [75.86395045428075, 22.6975729665635],
                    [75.88453790499463, 22.727188414518498],
                    [75.88446181010153, 22.719411637801603],
                    [75.87190266777337, 22.716151656831013],
                    [75.87138622544546, 22.72510570009874],
                    [75.8069296407894, 22.72934010670688]
                ]
            above is the list of locations needed to be visited by the salesman
            
      b. "num_vehicles": 1
            number of vehicles available to cover all location

2. For solving Vehicle routing problem following input is required:
    a. "list_cord": [
                    [75.85959398126745, 22.796033910515693],
                    [75.84514379661131, 22.797202783046455],
                    [75.89372268920312, 22.744358154533295],
                    [75.90357728311815, 22.754718648887533],
                    [75.88515698311818, 22.757859818243723],
                    [75.85584452544431, 22.67783817185593],
                    [75.87114646777299, 22.699688243109538],
                    [75.86395045428075, 22.6975729665635],
                    [75.88453790499463, 22.727188414518498],
                    [75.88446181010153, 22.719411637801603],
                    [75.87190266777337, 22.716151656831013],
                    [75.87138622544546, 22.72510570009874],
                    [75.8069296407894, 22.72934010670688]
                ],
    above is the list of locations needed to be visited by available salesman
    
    b. "num_vehicles": 3, 
    when the vehicle available is more than one we go for this solution so here we provide total number of vehicles available
    
    c. "vehicle_maximum_travel_distance": 40000 
    as the name suggested it is the contraint of a vehicle that how much maximum distance it can cover.
    
3. For solving Time window contraint problem we need following:
    
    a. "time_matrix": [
                    [0, 6, 9, 8, 7, 3, 6, 2, 3, 2, 6, 6, 4, 4, 5, 9, 7],
                    [6, 0, 8, 3, 2, 6, 8, 4, 8, 8, 13, 7, 5, 8, 12, 10, 14],
                    [9, 8, 0, 11, 10, 6, 3, 9, 5, 8, 4, 15, 14, 13, 9, 18, 9],
                    [8, 3, 11, 0, 1, 7, 10, 6, 10, 10, 14, 6, 7, 9, 14, 6, 16],
                    [7, 2, 10, 1, 0, 6, 9, 4, 8, 9, 13, 4, 6, 8, 12, 8, 14],
                    [3, 6, 6, 7, 6, 0, 2, 3, 2, 2, 7, 9, 7, 7, 6, 12, 8],
                    [6, 8, 3, 10, 9, 2, 0, 6, 2, 5, 4, 12, 10, 10, 6, 15, 5],
                    [2, 4, 9, 6, 4, 3, 6, 0, 4, 4, 8, 5, 4, 3, 7, 8, 10],
                    [3, 8, 5, 10, 8, 2, 2, 4, 0, 3, 4, 9, 8, 7, 3, 13, 6],
                    [2, 8, 8, 10, 9, 2, 5, 4, 3, 0, 4, 6, 5, 4, 3, 9, 5],
                    [6, 13, 4, 14, 13, 7, 4, 8, 4, 4, 0, 10, 9, 8, 4, 13, 4],
                    [6, 7, 15, 6, 4, 9, 12, 5, 9, 6, 10, 0, 1, 3, 7, 3, 10],
                    [4, 5, 14, 7, 6, 7, 10, 4, 8, 5, 9, 1, 0, 2, 6, 4, 8],
                    [4, 8, 13, 9, 8, 7, 10, 3, 7, 4, 8, 3, 2, 0, 4, 5, 6],
                    [5, 12, 9, 14, 12, 6, 6, 7, 3, 3, 4, 7, 6, 4, 0, 9, 2],
                    [9, 10, 18, 6, 8, 12, 15, 8, 13, 9, 13, 3, 4, 5, 9, 0, 9],
                    [7, 14, 9, 16, 14, 8, 5, 10, 6, 5, 4, 10, 8, 6, 2, 9, 0]
                ]
        it is the matrix which contain time required to reach from location a to b
        
        
    b. "time_windows": [
                    [0, 5],
                    [7, 12],
                    [10, 15],
                    [16, 18],
                    [10, 13],
                    [0, 5],
                    [5, 10],
                    [0, 4],
                    [5, 10],
                    [0, 3],
                    [10, 16],
                    [10, 15],
                    [0, 5],
                    [5, 10],
                    [7, 8],
                    [10, 15],
                    [11, 15]
                ]
    above input states the lime limit with in which salesman must reach that location
                
    c. "num_vehicles": 3, it is the total number of vehicles available for covering route
    
    d. "allow_waiting_time": 30, it is the maximum time a salesman can wait at a particular location
                
    e. "maximum_time_per_vehicle": 30, maximum time available per vehicle
    
    f. "depot": 0 It is the starting point of the routing
        
            


"""
class ObtainBestRoute(GenericAPIView):
    def get(self, request, *args, **kwargs):

        coordinate_list = request.data.get("list_cord", None)
        num_vehicles = request.data.get("num_vehicles", None)
        vehicle_maximum_travel_distance = request.data.get(
            "vehicle_maximum_travel_distance", None
        )
        time_matrix = request.data.get("time_matrix", None)
        time_windows = request.data.get("time_windows", None)

        if time_matrix and time_windows:

            route = RouteFinder(
                time_matrix=time_matrix,
                time_windows=time_windows,
                num_vehicles=4,
                allow_waiting_time=30,
                maximum_time_per_vehicle=30,
                depot=0,
            )
            logging.info(
                "Initiating Time Window Contraint Solution, Ready to find best route"
            )
            try:
                resp = route.time_window_constraint_solution()
                return Response(resp)
            except Exception as e:
                logging.info(
                    {
                        "message": "error occur while finding best route for Time Window Contraint problem is {}".format(
                            e
                        )
                    }
                )

        elif coordinate_list and num_vehicles:
            matrix = DistanceMatrix(coordinate_list)
            distance_data = matrix.create_distance_matrix()
            route = RouteFinder(
                distance_matrix=distance_data,
                coordinate_list=coordinate_list,
                num_vehicles=num_vehicles,
                depot=0,
                vehicle_maximum_travel_distance=vehicle_maximum_travel_distance,
            )
            try:
                resp = (
                    route.traveling_salesperson_solution()
                    if num_vehicles == 1
                    else route.vehicle_routing_solution()
                )
                return Response(resp)
            except Exception as e:
                logging.info(
                    {
                        "message": "error occur while finding best route for travelling salesman problem is {}".format(
                            e
                        )
                    }
                )

        else:
            logging.info("No Solution Found")
            return Response({"message": "No Solution Found"})
