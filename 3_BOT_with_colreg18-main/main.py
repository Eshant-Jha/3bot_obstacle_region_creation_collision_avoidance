#!/usr/bin/env python

"""
Created on Tue Jul 11 16:51:52 2023
Mobile robot simulation setup
@author: Eshant (OE22S300)
"""

#Import files
import search
import journal_code_1_orientation
import maze
import maze_map
import time

import sim_interface


def main():
    
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        robot1 = sim_interface.Pioneer(1)
        robot2 = sim_interface.Pioneer(2)   
        robot3 = sim_interface.Pioneer(3)   
        
        robot_list = [robot1,robot2,robot3]
        for robot in robot_list:
            robot.robot_list = robot_list
            
            
        #Start simulation
        if (sim_interface.start_simulation()):
         
            #Initialize the Maze
            current_maze = maze.Maze(1)
    
            #Obtain robot position
            robot1.localize_robot()
            robot2.localize_robot()
            robot3.localize_robot()
            
            #Update robot position in maze map            
            current_maze.maze_map.r1_start = [round(robot1.current_state[0]), round(robot1.current_state[1])]
            current_maze.maze_map.r2_start = [round(robot2.current_state[0]), round(robot2.current_state[1])]
            current_maze.maze_map.r3_start = [round(robot3.current_state[0]), round(robot3.current_state[1])]
            
            #Set goal locations
            current_maze.maze_map.r1_goal = [5,9]  #r1 start [5,1], r2_start [10,6] ,r3_start [2,4] positioned in scene 
            current_maze.maze_map.r2_goal = [1,7]
            current_maze.maze_map.r3_goal = [9,4]       
            
            #Plan the path
            robot1.path =  search.aStarSearch(current_maze, 1, current_maze.maze_map.r1_start, 1,[],[],[])
            robot2.path =  search.aStarSearch(current_maze, 2, current_maze.maze_map.r2_start, 1,[],[],[])
            robot3.path =  search.aStarSearch(current_maze, 3, current_maze.maze_map.r3_start, 1,[],[],[])
            
            r_path1=robot1.path
            r_path2=robot2.path
            r_path3=robot3.path
            
            #equalising path length by repeating goal value in shorter path
            
            journal_code_1_orientation.equalise_path(r_path1, r_path2, r_path3)
            print(r_path1,r_path2,r_path3,sep="\n\n")    
            
            #Setting local goals for each robot
            if r_path1 and r_path2 and r_path3:
               
                for i in range(max(len(r_path1),len(r_path2))):
                  print("Going to next waypoint... ",i)            
                  if i < len(r_path1):
                      
                      robot1.goal_state =  r_path1[i] 
                      current_maze.map_plot_copy[r_path1[i][0]][r_path1[i][1]] = maze_map.r1_path_id  
                      print(r_path1[i],"local goal for r1")    
                      
                  if i< len(r_path2) :
                      
                      robot2.goal_state   =  r_path2[i] 
                      current_maze.map_plot_copy[r_path2[i][0]][r_path2[i][1]] = maze_map.r2_path_id  
                      
                  if i< len(r_path3) :
                      
                      robot3.goal_state   =  r_path3[i]  
                      current_maze.map_plot_copy[r_path3[i][0]][r_path3[i][1]] = maze_map.r3_path_id  
                      
                  #Plot the solution
                  current_maze.plot_map()
                   
                  #Driving to the local goal
                  while not robot1.robot_at_goal() or not robot2.robot_at_goal() or not robot3.robot_at_goal(): 
                      
                    robot1_status = robot1.run_controller()
                    robot2_status = robot2.run_controller()
                    robot3_status = robot3.run_controller()
                    
                    if robot1_status==3:  #return 3 (from sim_interface file) # means collision detected
                        print(robot1.path,"r1 collison detected")
                        #updating all bot path if it was locally planned 
                        r_path1=robot1.path
                        r_path2=robot2.path
                        r_path3=robot3.path  
                       
                        break  #break will help to jump to next state of all bots
                   
                    if robot2_status==3:
                        print(" r2 collison detected")
                        #updating all bot path if it was locally planned 
                        r_path1=robot1.path
                        r_path2=robot2.path
                        r_path3=robot3.path
                        break
                                        
                    if  robot3_status==3:
                        print(robot3.path,"r3 collison detected")
                        #updating all bot path if it was locally planned 
                        r_path1=robot1.path
                        r_path2=robot2.path
                        r_path3=robot3.path
                        
                        break
                    
                    
                  #Stop robot                  
                  print("Reached local goal for all three robots")
                  robot1.setvel_pioneer(0.0, 0.0)
                  robot2.setvel_pioneer(0.0, 0.0)
                  robot3.setvel_pioneer(0.0, 0.0) 
                  
        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    #stop robots
    
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    return

#Run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            
