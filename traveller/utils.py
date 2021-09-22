"""Simple travelling salesman problem between cities."""
import math, logging
from math import cos
from math import sin
from math import asin
from math import sqrt
from math import radians

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


class RouteFinder:
    def __init__(
        self,
        distance_matrix=None,
        time_matrix=None,
        time_windows=None,
        coordinate_list=None,
        num_vehicles=1,
        depot=0,
        vehicle_maximum_travel_distance=3000,
        allow_waiting_time=30,
        maximum_time_per_vehicle=30,
    ):
        self.coordinate_list = coordinate_list
        self.data = {
            "distance_matrix": distance_matrix,
            "num_vehicles": num_vehicles if num_vehicles else 1,
            "depot": depot if depot else 0,
        }
        self.vehicle_maximum_travel_distance = vehicle_maximum_travel_distance
        self.allow_waiting_time = allow_waiting_time
        self.maximum_time_per_vehicle = maximum_time_per_vehicle
        self.response = {}

        self.data["time_matrix"] = time_matrix
        self.data["time_windows"] = time_windows

    def traveling_salesperson_response(self, manager, routing, solution):
        # self.response['Objective'] = '{} miles'.format(solution.ObjectiveValue())
        index = routing.Start(0)
        plan_output = "Route for vehicle: "
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += " {} ->".format(
                self.coordinate_list[manager.IndexToNode(index)]
            )
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        plan_output += " {}".format(self.coordinate_list[manager.IndexToNode(index)])
        self.response["plan_output"] = plan_output
        self.response["Route distance"] = "{} meters".format(route_distance)

    def traveling_salesperson_solution(self):
        """Entry point of the program."""
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(self.data["distance_matrix"]),
            self.data["num_vehicles"],
            self.data["depot"],
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return self.data["distance_matrix"][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            self.traveling_salesperson_response(manager, routing, solution)

        return self.response

    def vehicle_routing_response(self, manager, routing, solution):
        """Prints solution on console."""
        print(f"Objective: {solution.ObjectiveValue()}")
        max_route_distance = 0
        for vehicle_id in range(self.data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            plan_output = "Route: "
            route_distance = 0
            while not routing.IsEnd(index):
                plan_output += " {} -> ".format(
                    self.coordinate_list[manager.IndexToNode(index)]
                )
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id
                )
            plan_output += "{}".format(self.coordinate_list[manager.IndexToNode(index)])
            self.response["Route for vehicle {}".format(vehicle_id)] = plan_output
            self.response[
                "Distance of the route for vehicle {} in meter".format(vehicle_id)
            ] = route_distance
            max_route_distance = max(route_distance, max_route_distance)
        self.response["Maximum of the route distances"] = max_route_distance
        return self.response

    def vehicle_routing_solution(self):
        """Entry point of the program."""
        # Instantiate the data problem.

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(self.data["distance_matrix"]),
            self.data["num_vehicles"],
            self.data["depot"],
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Create and register a transit callback.
        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return self.data["distance_matrix"][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Distance constraint.
        dimension_name = "Distance"
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            self.vehicle_maximum_travel_distance,  # vehicle maximum travel distance
            True,  # start cumul to zero
            dimension_name,
        )
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            return self.vehicle_routing_response(manager, routing, solution)
        else:
            self.response["message"] = "No solution found !"
            return self.response

    def time_window_constraint_response(self, manager, routing, solution):

        self.response["Objective"] = solution.ObjectiveValue()
        time_dimension = routing.GetDimensionOrDie("Time")
        total_time = 0
        for vehicle_id in range(self.data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            plan_output = "Route :-->"
            while not routing.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
                plan_output += "{0} Time({1},{2}) -> ".format(
                    manager.IndexToNode(index),
                    solution.Min(time_var),
                    solution.Max(time_var),
                )
                index = solution.Value(routing.NextVar(index))
            time_var = time_dimension.CumulVar(index)
            plan_output += "{0} Time({1},{2})".format(
                manager.IndexToNode(index),
                solution.Min(time_var),
                solution.Max(time_var),
            )
            plan_output += "Time of the route: {}min".format(solution.Min(time_var))
            total_time += solution.Min(time_var)
            self.response["Route for vehicle {}".format(vehicle_id)] = plan_output
        self.response["Total time of all routes"] = total_time
        return self.response

    def time_window_constraint_solution(self):
        """Solve the VRP with time windows."""
        # Instantiate the data problem.
        # import pdb;pdb.set_trace()
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(self.data["time_matrix"]), self.data["num_vehicles"], self.data["depot"]
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Create and register a transit callback.
        def time_callback(from_index, to_index):
            """Returns the travel time between the two nodes."""
            # Convert from routing variable Index to time matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return self.data["time_matrix"][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(time_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Time Windows constraint.
        time = "Time"
        routing.AddDimension(
            transit_callback_index,
            self.allow_waiting_time,  # allow waiting time
            self.maximum_time_per_vehicle,  # maximum time per vehicle
            False,  # Don't force start cumul to zero.
            time,
        )
        time_dimension = routing.GetDimensionOrDie(time)
        # Add time window constraints for each location except depot.
        for location_idx, time_window in enumerate(self.data["time_windows"]):
            if location_idx == self.data["depot"]:
                continue
            index = manager.NodeToIndex(location_idx)
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        # Add time window constraints for each vehicle start node.
        depot_idx = self.data["depot"]
        for vehicle_id in range(self.data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            time_dimension.CumulVar(index).SetRange(
                self.data["time_windows"][depot_idx][0],
                self.data["time_windows"][depot_idx][1],
            )

        # Instantiate route start and end times to produce feasible times.
        for i in range(self.data["num_vehicles"]):
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.Start(i))
            )
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.End(i))
            )

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            return self.time_window_constraint_response(manager, routing, solution)
        else:
            self.response["message"] = "No Solution Found !"
            return self.response


class DistanceMatrix:
    def __init__(self, coordinate_sequence):
        self.source = coordinate_sequence
        self.destination = coordinate_sequence
        self.distance_matrix = []

    def haversine(self, pointA, pointB):
        if (type(pointA) != tuple) or (type(pointB) != tuple):
            raise TypeError("Only tuples are supported as arguments")
        lat1 = pointA[1]
        lon1 = pointA[0]
        lat2 = pointB[1]
        lon2 = pointB[0]
        # convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(
            radians, [float(lat1), float(lon1), float(lat2), float(lon2)]
        )
        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * asin(sqrt(a))
        r = 6371  # Radius of earth in kilometers. Use 3956 for miles
        # returns result in meter
        return c * r * 1000

    def sqrt_distance(self, pointA, pointB):
        lat1 = pointA[1]
        lon1 = pointA[0]
        lat2 = pointB[1]
        lon2 = pointB[0]
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        return math.sqrt(dlon ** 2 + dlat ** 2)

    def create_distance_matrix(self):
        for pointA in self.source:
            row = []
            for pointB in self.destination:
                row.append(self.haversine(tuple(pointA), tuple(pointB)))
            self.distance_matrix.append(row)
        return self.distance_matrix
