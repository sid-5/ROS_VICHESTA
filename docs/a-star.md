# A-star path finding algorithm

>A* is a graph traversal and path search algorithm, which is often used in many fields of computer science due to its completeness, optimality, and optimal efficiency.
>
>A* is an informed search algorithm, or a best-first search, meaning that it is formulated in terms of weighted graphs: starting from a specific starting node of a graph, it aims to find a path to the given goal node having the smallest cost (least distance travelled, shortest time, etc.). It does this by maintaining a tree of paths originating at the start node and extending those paths one edge at a time until its termination criterion is satisfied.
>
>At each iteration of its main loop, A* needs to determine which of its paths to extend. It does so based on the cost of the path and an estimate of the cost required to extend the path all the way to the goal. Specifically, A* selects the path that minimizes
>
><img src="https://user-images.githubusercontent.com/62329500/135568953-0a379697-0b61-476c-ba7b-4a933903b8b9.png">
>
>where n is the next node on the path, g(n) is the cost of the path from the start node to n, and h(n) is a heuristic function that estimates the cost of the cheapest path from n to the goal.
>
>A* terminates when the path it chooses to extend is a path from start to goal or if there are no paths eligible to be extended. The heuristic function is problem-specific. If the heuristic function is admissible, meaning that it never overestimates the actual cost to get to the goal, A* is guaranteed to return a least-cost path from start to goal.

><img src="https://user-images.githubusercontent.com/62329500/135568413-84269f97-428c-4439-b064-ce8608ac60ea.png" height="400" width="400">
>
>Consider the above scenario where robot wants to move from red block to green block
>
>The optimal path would be diagonally upright but due to the black obstacles the robot would have to evaluate other possible paths neighbouring to visited block with the next heuristically shorter path to reach the goal.
>
>There are 8 neighbours, added to an array, which would be sorted on total distance from start to goal with that block as an intermediate. An updation is performed on g(n) and f(n) gets affected as well. The array is then used as a queue to get the next best block to visit.
>
>Have a visual look at how the algorithm finds the path
>
>https://user-images.githubusercontent.com/62329500/133422506-26866080-b5fb-46b3-9601-ec52f603e244.mp4
>
