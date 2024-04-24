#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv

class Way_Make:
    def __init__(self):
        self.global_ref = self.read_csv('/home/agilex/taehun_limo/maps/way.csv', skip_header=True)
        self.global_ref_x = []
        self.global_ref_y = []
        
        self.edit_global_ref_x = []
        self.edit_global_ref_y = []
        
    def read_csv(self, file_path, skip_header=False):
        with open('/home/agilex/taehun_limo/maps/way.csv', 'r') as file:
            reader = csv.reader(file)
            if skip_header:
                next(reader)  # Skip the header row
            data = [list(map(float, row)) for row in reader]
        return np.array(data)
    
    def way_make(self):
        self.global_ref_x = self.global_ref[:, 0]
        self.global_ref_y = self.global_ref[:, 1]
        
        for i in range(0, len(self.global_ref), 3):
            self.edit_global_ref_x.append(self.global_ref_x[i])
            self.edit_global_ref_y.append(self.global_ref_y[i])
            
        # CSV 파일 작성
        with open("/home/agilex/taehun_limo/maps/way_edit.csv", 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['edit_global_ref_x', 'edit_global_ref_y'])
            for x, y in zip(self.edit_global_ref_x, self.edit_global_ref_y):
                writer.writerow([x, y])

def main():
    way = Way_Make()
    way.way_make()

if __name__ == "__main__":
    main()

