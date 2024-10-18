#!/bin/bash
# target_file="../../files/feature_desc/20231209_hoffice_med5_tf_stat_rem_filter_PointNormal.pcd"
# cfg_preset="-pcfg ../cfg/customPreset7.cfg"
#preset=custom5cuTest
preset=fpfh_4
c_preset=p2a
cfg_preset="-p ${preset}"
target_file_source="20231213_F602_highres_clip_table_tf"
target_file_lidar="../../files/${target_file_source}.pcd"
target_file="../../files/feature_desc/${target_file_source}_${preset}.pcd"
source_file_src="20240430_fhtw_f602_${c_preset}_"
source_file="../../files/${source_file_src}"
source_file_appendix="_filtered.pcd"
flist_output_file=allignment_successful_p5a_fpfh_5cu2.flist.tmp
flist_filename="../../files/file_lists/${source_file_src}${preset}.flist"
let threads=12

### set either sequential or custom file list
let custom_list=0 # 0 or 1

let start=0
let end=121

custom_list_entries=(123 125 128)
#custom_list_entries=(30) # 36 40 46 77 81 85 90 93 96 138 144 149)

if [ $custom_list -ne 1 ]; then
    ## sequential list
    capture_list=($(seq $start 1 $end))
    echo "using sequential file list ${capture_list[*]}"
else
    ## custom list
    capture_list=${custom_list_entries[*]}
    echo "using custom file list ${capture_list[*]}"
fi

echo "------- starting lidar preprocessing -------- ${cfg_preset}"
./14_lidar_preprocessing ${target_file_lidar} -t fpfh ${cfg_preset}
retVal=$?
#echo "------- Retval = ${retVal}"

if [ $retVal -ne 0 ]; then
    echo "------- ERROR: lidar preprocessing FAILED --------"
    exit -1
fi

echo "------- starting estimation -------- ${cfg_preset}"  >> ${flist_output_file}
echo "capture numbers: ${capture_list[*]}" >> ${flist_output_file}
echo ${target_file} >> ${flist_output_file}

for ((j=0;j<threads;j++))
do

#    for ((i=$start;i<=$end;i++))
    for i in ${capture_list[@]};
    do
        if [ $(($i%$threads)) -eq $j ]; then

            source_file_tmp=${source_file}
            source_file_tmp+=${i}
            source_file_tmp+=${source_file_appendix}
            let count_total++

            echo ""
            echo "--------------------------------------------------------------"
            echo "processing ${target_file} ${source_file_tmp}"
            echo "------- starting estimation -------- ${cfg_preset}"
        #    ./06_normal_fpfh ${target_file} ${source_file_tmp} ${cfg_preset}
            ./15_fpfh_registration ${target_file} ${source_file_tmp} ${cfg_preset} -log
            retVal=$?

            if [ $retVal -eq 0 ]; then
                echo "SUCCESSFUL estimation - writing to ${flist_output_file}"
                echo ${source_file_tmp} >> ${flist_output_file}
                let count_succ++
            fi
        fi
    done &

done

wait

# echo "------- estimation finished:  ${count_succ} of ${count_total} successful --------" >> ${flist_output_file}
echo "------- estimation finished -------" >> ${flist_output_file}

# printf "\n\nSUCCESSFUL estimations: %d of %d \n" ${count_succ} ${count_total}
printf "\n\nestimation finished\n"


echo "It's time to adapt your ${flist_filename} file"
powershell.exe '[console]::beep(880,1500)'
read -n1 -s -r -p $'Press any key to continue...\n' key

draw_pos_history_command="./08_draw_position_history ${flist_filename} -fpfh -pcd ${cfg_preset}"
echo "draw_pos_history_command: ${draw_pos_history_command}" >> ${flist_output_file}

# symbolic link in current directory required
echo executing: ${draw_pos_history_command}
${draw_pos_history_command}
