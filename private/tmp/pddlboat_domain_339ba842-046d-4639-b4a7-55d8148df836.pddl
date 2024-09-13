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
    (constraints)
    (emptyhand)
    (holding ?x - block)
    (on ?x - block ?y - block)
    (ontable ?x - block)
  )

  (:derived (constraints)
    (and
      (not 
        (holding b4))
      (not 
        (on b2 b3)))
  )

  (:action pick-up
    :parameters (?x - block)
    :precondition
      (and
        (and
          (clear ?x)
          (ontable ?x)
          (emptyhand))
        (constraints))
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
        (constraints))
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
        (and
          (holding ?x)
          (clear ?y))
        (constraints))
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
        (and
          (on ?x ?y)
          (clear ?x)
          (emptyhand))
        (constraints))
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