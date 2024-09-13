(define (domain blocks-domain)
  (:requirements
    :strips
    :typing
    :negative-preconditions
    :disjunctive-preconditions
    :derived-predicates
  )

  (:types
    block
  )

  (:predicates 
    (clear ?x - block)
    (emptyhand)
    (holding ?x - block)
    (on ?x - block ?y - block)
    (ontable ?x - block)
    (constraint-violated)
  )

  (:derived (constraint-violated)
    (or (holding b4) (on b1 b2))
  )

  (:action pick-up
    :parameters (?x - block)
    :precondition
      (and
        (clear ?x)
        (ontable ?x)
        (emptyhand)
        (not (constraint-violated)))
    :effect
      (and
        (not 
          (ontable ?x))
        (not 
          (clear ?x))
        (not 
          (emptyhand))
        (holding ?x))
  )

  (:action put-down
    :parameters (?x - block)
    :precondition
      (and 
        (holding ?x) 
        (not (constraint-violated)))
    :effect
      (and
        (not 
          (holding ?x))
        (clear ?x)
        (emptyhand)
        (ontable ?x))
  )

  (:action stack
    :parameters (?x - block ?y - block)
    :precondition
      (and
        (holding ?x)
        (clear ?y)
        (not (constraint-violated)))
    :effect
      (and
        (not 
          (holding ?x))
        (not 
          (clear ?y))
        (clear ?x)
        (emptyhand)
        (on ?x ?y))
  )

  (:action unstack
    :parameters (?x - block ?y - block)
    :precondition
      (and
        (on ?x ?y)
        (clear ?x)
        (emptyhand)
        (not (constraint-violated)))
    :effect
      (and
        (holding ?x)
        (clear ?y)
        (not 
          (clear ?x))
        (not 
          (emptyhand))
        (not 
          (on ?x ?y)))
  )

)