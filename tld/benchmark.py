#!/usr/bin/python

if __name__ == "__main__":
    import os

    datasets = (
	("01_david", 761),
	("02_jumping", 313),
	("03_pedestrian1", 140),
	("04_pedestrian2", 338),
	("05_pedestrian3", 184),
	("06_car", 945))

    method = "Dfbtrack"    
    ds_dir="/media/lab600/dataset/tld"
    out_dir = "../data/benchmark/tld_dataset/testing"
    
    cmd_tpl = "./fbtrack_cli -i %s/%%05d.jpg -s 1 -e %d -b %s > %s"
    for ds in datasets:        
        datapath = ds_dir + '/' + ds[0];
        f = open(datapath + '/init.txt', 'r')
        bb = f.readline()
        f.close()
        out_file = out_dir + '/' + ds[0] + ':' + method + '.txt'
        cmd = cmd_tpl%(datapath, ds[1], bb, out_file)
        
        os.system(cmd)
