# Project Setup
* Run ./manage.py runserver
* Go to the postman and run below url as **get** method
> http://127.0.0.1:8000/api/getroute/

* For travelling salesman problem Pass input in the body as shown below
> **{
                "list_cord": [
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
                "num_vehicles": 1,
            }**
* For vehicle routing problem Pass input in the body as shown below
> **{
                "list_cord": [
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
                "num_vehicles": 3,
                "vehicle_maximum_travel_distance": 40000
            }**

* For time window constraint problem Pass input in the body as shown below
> **{
                "time_matrix": [
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
                ],
                "time_windows": [
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
                ],
                "num_vehicles": 3,
                "allow_waiting_time": 30,
                "maximum_time_per_vehicle": 30,
                "depot": 0
            }**

* Run below command to test all unit cases
> **./manage.py test**
