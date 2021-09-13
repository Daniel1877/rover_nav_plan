(define (problem rover_problem)
   (:domain rover)
   (:objects punto0 punto1 punto2 punto3 punto4 punto5 punto6 - points
             r - rover
             loc_cam_depth nav_cam_depth - camera
             )

  (:init  (at punto0)

          (free r)

          (move-possible punto0 punto1)
          (move-possible punto1 punto2)
          (move-possible punto2 punto3)
          (move-possible punto3 punto4)
          (move-possible punto4 punto5)
          (move-possible punto5 punto6)

          (= (distance punto0 punto1) 10)
          (= (distance punto1 punto2) 20)
          (= (distance punto2 punto3) 20)
          (= (distance punto3 punto4) 7)
          (= (distance punto4 punto5) 18)
          (= (distance punto5 punto6) 25)

          (= (battery r) 20)
          (= (consum r) 1)
          (= (capacity r) 40)
          (= (recharge-rate r) 2)
          (= (velocity r) 2)
          (= (photo-energy r) 5)
          (= (calibration-energy r) 10)
   )

   (:goal (and  (photography punto1 loc_cam_depth)
                (photography punto2 nav_cam_depth)
                (photography punto4 nav_cam_depth)
                (photography punto5 loc_cam_depth)
                (photography punto6 nav_cam_depth)
                (at punto6)
          )
   )
)
