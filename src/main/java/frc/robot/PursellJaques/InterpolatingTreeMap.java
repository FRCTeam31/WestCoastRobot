// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PursellJaques;

import java.util.TreeMap;

/** Add your docs here. */
public class InterpolatingTreeMap {
    // Instance Variables
    private TreeMap<Double, Double>  map;

    public InterpolatingTreeMap(){
        map = new TreeMap<Double, Double>();
    }

    /**
     * Add a key-value pair to the tree map
     * @param key
     * @param value
     */
    public void put(Double key, Double value){
        map.put(key, value);
    }

    /**
     * Get a value that is interpolated from the nearest 2 values
     * Returns 0.0 if the tree is empty 
     * @param key 
     * @return Interpolated value
     */
    public double getInterpolatedValue(Double key){
        if(map.size() == 0){
            return 0.0;
        }
        
        Double ceilingKey = map.ceilingKey(key);
        Double floorKey = map.floorKey(key);
        double value;

        // Interpolate
        if(ceilingKey == null){
            // There is no higher key, use the max key 
            value = map.get(map.lastKey());
        }
        else if(floorKey == null){
            // There is no lower key, use the lowest key
            value = map.get(map.firstKey());
        }
        else{
            // Key is somehwere in the middle, interpolate
            double keyDifference = ceilingKey - floorKey;
            value = (ceilingKey - key) / keyDifference * ((double) map.get(floorKey)) + (key - floorKey) / keyDifference * ((double) map.get(ceilingKey));
        }

        return value;
    }
}
