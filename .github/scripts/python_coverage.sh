#!/bin/bash

catkin_build_dir=$1

files_to_process=""


coverage_file_basename=".coverage_record_";
counter=0;

for cov_file in $(find ${HOME}/.ros -name "*.coverage*"); do
  echo "Copying coverage file '$cov_file'";
  copy_name="$coverage_file_basename$counter";
  files_to_process="$files_to_process $copy_name";
  counter=$((counter+1));
  cp "$cov_file" ./"$copy_name" ;
done

for cov_file in $(find "$catkin_build_dir" -name "*.coverage*"); do
  echo "Copying coverage file '$cov_file'";
  copy_name="$coverage_file_basename$counter";
  files_to_process="$files_to_process $copy_name";
  counter=$((counter+1));
  cp "$cov_file" ./"$copy_name" ;
done

python3-coverage combine -a $files_to_process
python3-coverage xml --omit "*/usr*","*/opt*","*/devel*","*test/*","*tests/*","*/.local*" -o coverage_python.xml 
