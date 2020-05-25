#!/bin/bash

###############################################################################
# run_tests.bash -- Dana Nau <nau@cs.umd.edu>, Sept 10, 2018
#
# This bash script uses three lists -- racetrack problems, heuristic functions,
# and search strategies -- and iterates over all possible combinations. For each
# combination, it does (1) a graphic demo of the search and (2) a time test.
#
# You should modify this file to use the sample problems, heuristics, and
# search strategies that you want to use.
###############################################################################


set -f    # disable globbing, because we don't want the name a* to be expanded

###############################################################################
### START OF CUSTOMIZATION OPTIONS


### IMPORTANT!!! Change this to use the pathname for your python program
#
python=/Library/Frameworks/Python.framework/Versions/3.6/bin/python3    # python pathname


#### TEST PROBLEMS ###

# Specify the rootname of the file that contains the test problems
#
prob_file=sample_probs

# Below, the first line is a list of all the problems in sample_probs.py, 
# in roughly increasing  order of difficulty. You should modify the second
# line to include just the one(s) you want to use.

problems=(rect20a rect20b rect20c rect20d rect50 wall8a wall8b lhook16 rhook16a rhook16b spiral16 spiral24 pdes30 pdes30b rectwall8 rectwall16 rectwall32 rectwall32a walls32)

problems=(rect20e lhook16)

#### HEURISTIC FUNCTIONS ###

# Here's the rootname of the file that contains the heuristic functions.
# You should replace it with the rootname of YOUR hueuristic function file
#
heur_file=proj1

# Below, the first line is a list of all the heuristic functions in racetrack.py.
# You should modify the second line to include just the one(s) you want to use.

heuristics=(h_edist h_esdist h_walldist)

heuristics=(proj1 h_walldist)


#### SEARCH STRATEGIES AND OTHER FLAGS ###

# Below, the first line is a list of all the search strategies in heuristics.py. 
# You should modify the second line to include just the one(s) you want to use.

strategies=(bf df uc gbf a*)

strategies=(gbf)

# Specify the amount of verbosity you want fsearch.main to use:
verb=2       

# Use draw=1 to draw the search tree; use draw=0 to draw nothing
draw=1  


### End of customization options
#####################################################################


# The following code will iterate over every combination of sample problem, 
# heuristic function, and search strategy in your above lists. For each combination,
# it will first display the results graphically, and then do a time test.
#
for prob in ${problems[*]}
do
  for heur in ${heuristics[*]}
  do
    for strat in ${strategies[*]}
    do
        echo ''
        echo "Graphic display of '$strat, $heur, $prob"

        # strings for preliminary setup, running the program, and printing the result
        setup="import racetrack, $prob_file, $heur_file"
        run_prog="result=racetrack.main($prob_file.$prob, '$strat', $heur_file.$heur, verbose=$verb, draw=$draw, title='$strat, $heur, $prob')"
        print_result="print('\nResult ({} states):\n{}'.format(len(result), result))"
        
        # execute the strings in python
        $python -c "$setup; $run_prog; $print_result"


        # Here's the code for doing a time test.
        echo ''
        echo "Time test of '$strat, $heur, $prob'"
        
        # string to do a time test (without printing or drawing anything)
        time_test="racetrack.main($prob_file.$prob, '$strat', $heur_file.$heur, verbose=0, draw=0)"

        # execute the time test
        $python -m timeit -s "$setup" "$time_test"
    done
  done
done
