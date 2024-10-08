(define (domain Rover)
    (:requirements :conditional-effects :derived-predicates :negative-preconditions :typing)
    (:types camera lander mode objective rover store waypoint)
    (:predicates (Y-Ocommunicated_rock_data-waypoint0) (Y-Ocommunicated_rock_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint2) (at ?x - rover ?y - waypoint)  (at_lander ?x - lander ?y - waypoint)  (at_rock_sample ?w - waypoint)  (at_soil_sample ?w - waypoint)  (available ?r - rover)  (calibrated ?c - camera ?r - rover)  (calibration_target ?i - camera ?o - objective)  (can_traverse ?r - rover ?x - waypoint ?y - waypoint)  (channel_free ?l - lander)  (communicated_image_data ?o - objective ?m - mode)  (communicated_rock_data ?w - waypoint)  (communicated_soil_data ?w - waypoint)  (empty ?s - store)  (equipped_for_imaging ?r - rover)  (equipped_for_rock_analysis ?r - rover)  (equipped_for_soil_analysis ?r - rover)  (full ?s - store)  (have_image ?r - rover ?o - objective ?m - mode)  (have_rock_analysis ?r - rover ?w - waypoint)  (have_soil_analysis ?r - rover ?w - waypoint)  (on_board ?i - camera ?r - rover)  (store_of ?s - store ?r - rover)  (supports ?c - camera ?m - mode)  (val-Ocommunicated_rock_data-waypoint0) (val-Ocommunicated_rock_data-waypoint1) (val-Ocommunicated_soil_data-waypoint1) (val-Ocommunicated_soil_data-waypoint1-or-Ocommunicated_soil_data-waypoint2-or-Ocommunicated_rock_data-waypoint0-or-Ocommunicated_rock_data-waypoint1) (val-Ocommunicated_soil_data-waypoint2) (val-communicated_rock_data-waypoint0) (val-communicated_rock_data-waypoint1) (val-communicated_soil_data-waypoint1) (val-communicated_soil_data-waypoint1-or-communicated_soil_data-waypoint2-or-communicated_rock_data-waypoint0-or-communicated_rock_data-waypoint1) (val-communicated_soil_data-waypoint1-or-communicated_soil_data-waypoint2-or-communicated_rock_data-waypoint0-or-communicated_rock_data-waypoint1-and-Ocommunicated_soil_data-waypoint1-or-Ocommunicated_soil_data-waypoint2-or-Ocommunicated_rock_data-waypoint0-or-Ocommunicated_rock_data-waypoint1) (val-communicated_soil_data-waypoint2) (visible ?w - waypoint ?p - waypoint)  (visible_from ?o - objective ?w - waypoint))
    (:derived (val-Ocommunicated_rock_data-waypoint0) (or (val-communicated_rock_data-waypoint0) (Y-Ocommunicated_rock_data-waypoint0)))
     (:derived (val-Ocommunicated_rock_data-waypoint1) (or (val-communicated_rock_data-waypoint1) (Y-Ocommunicated_rock_data-waypoint1)))
     (:derived (val-Ocommunicated_soil_data-waypoint1) (or (val-communicated_soil_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint1)))
     (:derived (val-Ocommunicated_soil_data-waypoint1-or-Ocommunicated_soil_data-waypoint2-or-Ocommunicated_rock_data-waypoint0-or-Ocommunicated_rock_data-waypoint1) (or (val-Ocommunicated_soil_data-waypoint1) (val-Ocommunicated_soil_data-waypoint2) (val-Ocommunicated_rock_data-waypoint0) (val-Ocommunicated_rock_data-waypoint1)))
     (:derived (val-Ocommunicated_soil_data-waypoint2) (or (val-communicated_soil_data-waypoint2) (Y-Ocommunicated_soil_data-waypoint2)))
     (:derived (val-communicated_rock_data-waypoint0) (communicated_rock_data waypoint0))
     (:derived (val-communicated_rock_data-waypoint0) (communicated_rock_data waypoint0))
     (:derived (val-communicated_rock_data-waypoint1) (communicated_rock_data waypoint1))
     (:derived (val-communicated_rock_data-waypoint1) (communicated_rock_data waypoint1))
     (:derived (val-communicated_soil_data-waypoint1) (communicated_soil_data waypoint1))
     (:derived (val-communicated_soil_data-waypoint1) (communicated_soil_data waypoint1))
     (:derived (val-communicated_soil_data-waypoint1-or-communicated_soil_data-waypoint2-or-communicated_rock_data-waypoint0-or-communicated_rock_data-waypoint1) (or (val-communicated_soil_data-waypoint1) (val-communicated_soil_data-waypoint2) (val-communicated_rock_data-waypoint0) (val-communicated_rock_data-waypoint1)))
     (:derived (val-communicated_soil_data-waypoint1-or-communicated_soil_data-waypoint2-or-communicated_rock_data-waypoint0-or-communicated_rock_data-waypoint1-and-Ocommunicated_soil_data-waypoint1-or-Ocommunicated_soil_data-waypoint2-or-Ocommunicated_rock_data-waypoint0-or-Ocommunicated_rock_data-waypoint1) (and (val-communicated_soil_data-waypoint1-or-communicated_soil_data-waypoint2-or-communicated_rock_data-waypoint0-or-communicated_rock_data-waypoint1) (val-Ocommunicated_soil_data-waypoint1-or-Ocommunicated_soil_data-waypoint2-or-Ocommunicated_rock_data-waypoint0-or-Ocommunicated_rock_data-waypoint1)))
     (:derived (val-communicated_soil_data-waypoint2) (communicated_soil_data waypoint2))
     (:derived (val-communicated_soil_data-waypoint2) (communicated_soil_data waypoint2))
    (:action calibrate
        :parameters (?r - rover ?i - camera ?t - objective ?w - waypoint)
        :precondition (and (equipped_for_imaging ?r) (calibration_target ?i ?t) (at ?r ?w) (visible_from ?t ?w) (on_board ?i ?r))
        :effect (and (calibrated ?i ?r) (when (val-Ocommunicated_rock_data-waypoint0) (Y-Ocommunicated_rock_data-waypoint0)) (when (not (val-Ocommunicated_rock_data-waypoint0)) (not (Y-Ocommunicated_rock_data-waypoint0))) (when (not (val-Ocommunicated_rock_data-waypoint1)) (not (Y-Ocommunicated_rock_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint1)) (not (Y-Ocommunicated_soil_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint2)) (not (Y-Ocommunicated_soil_data-waypoint2))) (when (val-Ocommunicated_rock_data-waypoint1) (Y-Ocommunicated_rock_data-waypoint1)) (when (val-Ocommunicated_soil_data-waypoint2) (Y-Ocommunicated_soil_data-waypoint2)) (when (val-Ocommunicated_soil_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint1)))
    )
     (:action communicate_image_data
        :parameters (?r - rover ?l - lander ?o - objective ?m - mode ?x - waypoint ?y - waypoint)
        :precondition (and (at ?r ?x) (at_lander ?l ?y) (have_image ?r ?o ?m) (visible ?x ?y) (available ?r) (channel_free ?l))
        :effect (and (not (available ?r)) (not (channel_free ?l)) (channel_free ?l) (communicated_image_data ?o ?m) (available ?r) (when (val-Ocommunicated_rock_data-waypoint0) (Y-Ocommunicated_rock_data-waypoint0)) (when (not (val-Ocommunicated_rock_data-waypoint0)) (not (Y-Ocommunicated_rock_data-waypoint0))) (when (not (val-Ocommunicated_rock_data-waypoint1)) (not (Y-Ocommunicated_rock_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint1)) (not (Y-Ocommunicated_soil_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint2)) (not (Y-Ocommunicated_soil_data-waypoint2))) (when (val-Ocommunicated_rock_data-waypoint1) (Y-Ocommunicated_rock_data-waypoint1)) (when (val-Ocommunicated_soil_data-waypoint2) (Y-Ocommunicated_soil_data-waypoint2)) (when (val-Ocommunicated_soil_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint1)))
    )
     (:action communicate_rock_data
        :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
        :precondition (and (at ?r ?x) (at_lander ?l ?y) (have_rock_analysis ?r ?p) (visible ?x ?y) (available ?r) (channel_free ?l))
        :effect (and (not (available ?r)) (not (channel_free ?l)) (channel_free ?l) (communicated_rock_data ?p) (available ?r) (when (val-Ocommunicated_rock_data-waypoint0) (Y-Ocommunicated_rock_data-waypoint0)) (when (not (val-Ocommunicated_rock_data-waypoint0)) (not (Y-Ocommunicated_rock_data-waypoint0))) (when (not (val-Ocommunicated_rock_data-waypoint1)) (not (Y-Ocommunicated_rock_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint1)) (not (Y-Ocommunicated_soil_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint2)) (not (Y-Ocommunicated_soil_data-waypoint2))) (when (val-Ocommunicated_rock_data-waypoint1) (Y-Ocommunicated_rock_data-waypoint1)) (when (val-Ocommunicated_soil_data-waypoint2) (Y-Ocommunicated_soil_data-waypoint2)) (when (val-Ocommunicated_soil_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint1)))
    )
     (:action communicate_soil_data
        :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
        :precondition (and (at ?r ?x) (at_lander ?l ?y) (have_soil_analysis ?r ?p) (visible ?x ?y) (available ?r) (channel_free ?l))
        :effect (and (not (available ?r)) (not (channel_free ?l)) (channel_free ?l) (communicated_soil_data ?p) (available ?r) (when (val-Ocommunicated_rock_data-waypoint0) (Y-Ocommunicated_rock_data-waypoint0)) (when (not (val-Ocommunicated_rock_data-waypoint0)) (not (Y-Ocommunicated_rock_data-waypoint0))) (when (not (val-Ocommunicated_rock_data-waypoint1)) (not (Y-Ocommunicated_rock_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint1)) (not (Y-Ocommunicated_soil_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint2)) (not (Y-Ocommunicated_soil_data-waypoint2))) (when (val-Ocommunicated_rock_data-waypoint1) (Y-Ocommunicated_rock_data-waypoint1)) (when (val-Ocommunicated_soil_data-waypoint2) (Y-Ocommunicated_soil_data-waypoint2)) (when (val-Ocommunicated_soil_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint1)))
    )
     (:action drop
        :parameters (?x - rover ?y - store)
        :precondition (and (store_of ?y ?x) (full ?y))
        :effect (and (not (full ?y)) (empty ?y) (when (val-Ocommunicated_rock_data-waypoint0) (Y-Ocommunicated_rock_data-waypoint0)) (when (not (val-Ocommunicated_rock_data-waypoint0)) (not (Y-Ocommunicated_rock_data-waypoint0))) (when (not (val-Ocommunicated_rock_data-waypoint1)) (not (Y-Ocommunicated_rock_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint1)) (not (Y-Ocommunicated_soil_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint2)) (not (Y-Ocommunicated_soil_data-waypoint2))) (when (val-Ocommunicated_rock_data-waypoint1) (Y-Ocommunicated_rock_data-waypoint1)) (when (val-Ocommunicated_soil_data-waypoint2) (Y-Ocommunicated_soil_data-waypoint2)) (when (val-Ocommunicated_soil_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint1)))
    )
     (:action navigate
        :parameters (?x - rover ?y - waypoint ?z - waypoint)
        :precondition (and (can_traverse ?x ?y ?z) (available ?x) (at ?x ?y) (visible ?y ?z))
        :effect (and (not (at ?x ?y)) (at ?x ?z) (when (val-Ocommunicated_rock_data-waypoint0) (Y-Ocommunicated_rock_data-waypoint0)) (when (not (val-Ocommunicated_rock_data-waypoint0)) (not (Y-Ocommunicated_rock_data-waypoint0))) (when (not (val-Ocommunicated_rock_data-waypoint1)) (not (Y-Ocommunicated_rock_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint1)) (not (Y-Ocommunicated_soil_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint2)) (not (Y-Ocommunicated_soil_data-waypoint2))) (when (val-Ocommunicated_rock_data-waypoint1) (Y-Ocommunicated_rock_data-waypoint1)) (when (val-Ocommunicated_soil_data-waypoint2) (Y-Ocommunicated_soil_data-waypoint2)) (when (val-Ocommunicated_soil_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint1)))
    )
     (:action sample_rock
        :parameters (?x - rover ?s - store ?p - waypoint)
        :precondition (and (at ?x ?p) (at_rock_sample ?p) (equipped_for_rock_analysis ?x) (store_of ?s ?x) (empty ?s))
        :effect (and (not (empty ?s)) (full ?s) (have_rock_analysis ?x ?p) (not (at_rock_sample ?p)) (when (val-Ocommunicated_rock_data-waypoint0) (Y-Ocommunicated_rock_data-waypoint0)) (when (not (val-Ocommunicated_rock_data-waypoint0)) (not (Y-Ocommunicated_rock_data-waypoint0))) (when (not (val-Ocommunicated_rock_data-waypoint1)) (not (Y-Ocommunicated_rock_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint1)) (not (Y-Ocommunicated_soil_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint2)) (not (Y-Ocommunicated_soil_data-waypoint2))) (when (val-Ocommunicated_rock_data-waypoint1) (Y-Ocommunicated_rock_data-waypoint1)) (when (val-Ocommunicated_soil_data-waypoint2) (Y-Ocommunicated_soil_data-waypoint2)) (when (val-Ocommunicated_soil_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint1)))
    )
     (:action sample_soil
        :parameters (?x - rover ?s - store ?p - waypoint)
        :precondition (and (at ?x ?p) (at_soil_sample ?p) (equipped_for_soil_analysis ?x) (store_of ?s ?x) (empty ?s))
        :effect (and (not (empty ?s)) (full ?s) (have_soil_analysis ?x ?p) (not (at_soil_sample ?p)) (when (val-Ocommunicated_rock_data-waypoint0) (Y-Ocommunicated_rock_data-waypoint0)) (when (not (val-Ocommunicated_rock_data-waypoint0)) (not (Y-Ocommunicated_rock_data-waypoint0))) (when (not (val-Ocommunicated_rock_data-waypoint1)) (not (Y-Ocommunicated_rock_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint1)) (not (Y-Ocommunicated_soil_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint2)) (not (Y-Ocommunicated_soil_data-waypoint2))) (when (val-Ocommunicated_rock_data-waypoint1) (Y-Ocommunicated_rock_data-waypoint1)) (when (val-Ocommunicated_soil_data-waypoint2) (Y-Ocommunicated_soil_data-waypoint2)) (when (val-Ocommunicated_soil_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint1)))
    )
     (:action take_image
        :parameters (?r - rover ?p - waypoint ?o - objective ?i - camera ?m - mode)
        :precondition (and (calibrated ?i ?r) (on_board ?i ?r) (equipped_for_imaging ?r) (supports ?i ?m) (visible_from ?o ?p) (at ?r ?p))
        :effect (and (have_image ?r ?o ?m) (not (calibrated ?i ?r)) (when (val-Ocommunicated_rock_data-waypoint0) (Y-Ocommunicated_rock_data-waypoint0)) (when (not (val-Ocommunicated_rock_data-waypoint0)) (not (Y-Ocommunicated_rock_data-waypoint0))) (when (not (val-Ocommunicated_rock_data-waypoint1)) (not (Y-Ocommunicated_rock_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint1)) (not (Y-Ocommunicated_soil_data-waypoint1))) (when (not (val-Ocommunicated_soil_data-waypoint2)) (not (Y-Ocommunicated_soil_data-waypoint2))) (when (val-Ocommunicated_rock_data-waypoint1) (Y-Ocommunicated_rock_data-waypoint1)) (when (val-Ocommunicated_soil_data-waypoint2) (Y-Ocommunicated_soil_data-waypoint2)) (when (val-Ocommunicated_soil_data-waypoint1) (Y-Ocommunicated_soil_data-waypoint1)))
    )
)