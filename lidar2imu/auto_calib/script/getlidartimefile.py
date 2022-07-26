#!/usr/bin/env python

import sys
import os.path

if __name__ == "__main__":

    if len(sys.argv) != 3:
        print ("usage: python3 get*.py <lidar_file_path> <output_txt_name>")
        sys.exit(1)

    BASE_PATH=sys.argv[1].split('.txt')[0]
    OUT_PUT_FILE = sys.argv[2].split('.txt')[0]
    file_out = open(OUT_PUT_FILE + ".txt", 'w')
    file_out_fullname = open(OUT_PUT_FILE + '_fullname'+ ".txt", 'w')
    files = os.listdir(BASE_PATH)
    files.sort()
    demo_pcd = ""
    initBool = True

    for filename in files:
        if filename[-4:] == ".pcd":
            if initBool:
                initBool=False
                demo_pcd = filename
            try:
                time_stamp = float(filename[:-4])*1e-7
                print("time stamp:\t"+ filename[:-4])
                file_out.write(str(time_stamp))
                file_out.write("\n")
                file_out_fullname.write(filename[:-4])
                file_out_fullname.write("\n")
            except ValueError:
                pass
    file_out.close()
    file_out_fullname.close()

