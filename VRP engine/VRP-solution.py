import herepy
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

API_KEY = 'PPQfDwuZL0HpO67O1KrsO5PSt91YjW-0my38saBczB8'

addresses = ['Klanjecka ulica, 10110 Zagreb, Croatia',
             'Prilaz Baruna Filipovića 22E, HR-10000 Zagreb, Croatia',
             'Ulica Franje Ogulinca-Selje 3, HR-10000 Zagreb, Croatia',
             'Ulica Branka Klarića, HR-10000 Zagreb, Croatia',
             'Ulica Antuna Šoljana, HR-10090 Zagreb, Croatia',
             'Rukavec 9, HR-10090 Zagreb, Croatia',
             'Ulica Antuna Šoljana, HR-10090 Zagreb, Croatia']

def computeDistanceMatrix():
    routingApi = herepy.routing_api.RoutingApi(API_KEY)
    response = routingApi.matrix(
        start_waypoints=addresses,
        destination_waypoints=addresses,
        modes=[herepy.RouteMode.shortest, herepy.RouteMode.car],
        summary_attributes=[herepy.here_enum.MatrixSummaryAttribute.distance])
    response = response.as_dict()

    noOfAddresses = len(addresses)
    distanceMatrix = [[0 for x in range(noOfAddresses)] for y in range(noOfAddresses)]
    for i in range(noOfAddresses):
        for j in range(noOfAddresses):
            if i != j:
                distanceMatrix[i][j] = response['response']['matrixEntry'][i * noOfAddresses + j]['summary']['distance']

    return distanceMatrix


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = computeDistanceMatrix()
    data['addresses'] = addresses
    data['num_vehicles'] = 3
    data['depot'] = 0
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(data['addresses'][manager.IndexToNode(index)])
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}m'.format(max_route_distance))


if __name__ == '__main__':
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]


    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        70000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
