
(define (domain dir2) 
  (:requirements
    :strips                 
    :negative-preconditions 
    :equality              
    :typing 
            
  )
(:types

    waypoints
   
)
 
  (:predicates
     (robot_at ?wp - waypoints)
     (obstacle ?wp - waypoints)
     (visited ?wp - waypoints)
     (grid ?wp - waypoints)
     (connected ?wp ?wp2 - waypoints)
  )

   (:action turtlebot_move
    :parameters (?from ?to - waypoints)
   
    :precondition (and
       (grid ?from) (grid ?to)(connected ?from ?to)(robot_at ?from)  (not (obstacle ?to)) (not (visited ?to))
    )
  
    :effect (and
      (not (robot_at ?from)) (robot_at ?to) (visited ?to)
     
    )
  )


)
