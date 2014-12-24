
# HEADER
'''   
@file:          kdtree_test.py
@brief:    	    This program tests the ability of data mining engine class kdtree.py in finding required configurations
@author:        Nima Ramezani; DFKI Bremen
@start date:    March 2010
@version:	    0.1
Last Revision:  20 April 2011
'''
# BODY

import __init__, kdtree, numpy
__init__.set_file_path(False)

import projects.dfki.nima.robotics.kinematics.kinematicpy.manipulator_library as maniplib
import projects.dfki.nima.robotics.kinematics.task_space.workspace as wslib 

import packages.dfki.bebo.vibot.math_logic.boolean.relations.relation as rels

def tuple_error(t1, t2):
    
    i = 0
    s = 0.0
    for x in t1:
        s += (x - t2[i]) ** 2
        i += 1
        
    return s
        

def main():

    manip = maniplib.select_manipulator('PUMA')
    
    manip.inverse_kinematics.endeffector.taskframes = []

    # Take manipulator to some arbitrary configuration in the jointspace grid
    manip.inverse_kinematics.take_to_grided_configuration(manip.geometry, config_number = 12129, number_of_intervals = 7)
    manip.inverse_kinematics.endeffector.update(manip.inverse_kinematics)
    
    # Set endeffector target as the pose corresponding to this arbitrary configuraton
    for tp in manip.inverse_kinematics.endeffector.taskpoints:
        tp.rd = numpy.copy(tp.ra)
    for tf in manip.inverse_kinematics.endeffector.taskframes:
        tf.rd = numpy.copy(tf.ra)

    wsp = wslib.Workspace()
    wsp.number_of_divisions = 3
    wsp.evaluate(manip.geometry, manip.inverse_kinematics.configuration, manip.inverse_kinematics.endeffector)

    pose_set = set(wsp.pose_list)

    tree = kdtree.KDTree.construct_from_data( list(pose_set)  )  
    
    # if "wsp.pose_list" is passed to method "kdtree.KDTree.construct_from_data", then the order of elements of "wsp.pose_list" changes and they match no more to "wsp.config_list"
    # So a copy of this list shuld be passed to the method
    # To understand this, try activating the following code:
    
    #tree = kdtree.KDTree.construct_from_data( wsp.pose_list )  
    
    #manip.inverse_kinematics.configuration.q = numpy.copy(wsp.config_list[591])
    #manip.inverse_kinematics.forward_update(manip.geometry)
    #manip.inverse_kinematics.endeffector.update(manip.inverse_kinematics)
    
    #print 'wsp.pose_list[591] = ', wsp.pose_list[591]
    #print 'The actual pose    = ', manip.inverse_kinematics.endeffector.taskpoints[0].ra

    nearest = tree.query(query_point = wsp.target_pose, t=10)

    manip.inverse_kinematics.endeffector.tasklist        = ['update_taskspace_pose', 'update_error']


    print 'len(pose_set)        = ',len(pose_set) 
    print 'len(wsp.pose_list)   = ',len(wsp.pose_list)
    print 
    print 'len(wsp.config_list) = ',len(wsp.config_list)
    print 'len(nearest)         = ',len(nearest)
    
    config_tuple_list = [ tuple(c) for c in wsp.config_list  ]
    d = dict( zip( config_tuple_list, wsp.pose_list) )
    f = rels.Function( d )
    configs_66 = f.preimage( wsp.pose_list[66] ) 
    print "configs_66"
    print configs_66
    print 
    configs_93 = f.preimage( wsp.pose_list[93] ) 
    print "configs_93"
    print configs_93
    print 
    
    print "---"

    
#    set_nearest = set(nearest) 
    
 #   print len(nearest)
  #  print len(set_nearest)
    
    
    # for pose in nearest:
    #    print wsp.pose_list.index(pose)
    #    print 'tuple Norm: ', tuple_error(pose , wsp.target_pose)
    #    print
    
    for pose in nearest:
        print 'This is the nearest pose     : ', pose
        print 'This is the target  pose     : ', wsp.target_pose
        
        print 'The index of the nearest pose: ', wsp.pose_list.index(pose)
        manip.inverse_kinematics.configuration.q = numpy.copy(wsp.config_list[wsp.pose_list.index(pose)])
        manip.inverse_kinematics.forward_update(manip.geometry)
        manip.inverse_kinematics.endeffector.update(manip.inverse_kinematics)
        
        print 'This is the nearest pose: ', manip.inverse_kinematics.endeffector.taskpoints[0].ra
        print 'This is the target  pose: ', manip.inverse_kinematics.endeffector.taskpoints[0].rd
        
        print
        print 'Error Norm: ', manip.inverse_kinematics.endeffector.error_norm
        print 'tuple Norm: ', tuple_error(pose , wsp.target_pose)
        print
      


main()
