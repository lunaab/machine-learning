# -*- coding: utf-8 -*-
"""
Created on Fri Oct  7 10:20:26 2016

@author: lunaab
"""
import numpy as np
import matplotlib as plt
from matplotlib import pyplot
import math as m
from scipy import misc
import sys

class pca:
    
    def __init__( self, file_name ):
        self.file_name = file_name
        self.holding_cell = None
        self.small_eigen = None
        self.occupied = 0
        self.image_row = 0
        self.image_col = 0
        self.row_count = 0
        self.col_count = 0
        
    def format_main_image( self, color_array ):
        self.new_array = (color_array[:,:,0] +  color_array[:,:,1] +  color_array[:,:,2]) / 3.0
        print self.new_array



        
        # (row, column, useless) = color_array.shape
        # self.image_row = row
        # self.image_col = column
        # new_array = np.zeros( ( row, column ) )
        
        # for i in range( 0, row - 1 ):
        #     for j in range( 0, column - 1 ):
        #         new_array[i][j] = color_array[i][j][1]
                
        return self.new_array
    
    def get_main_image(self):
        lion = misc.imread(self.file_name)
        pyplot.imshow(lion)
        return lion
        
        
    
    def get_sigma_matrix(self):
        (row, col) = self.holding_cell.shape

        self.x_mean = np.mean(self.holding_cell, axis=1)
        print 'xshape', self.x_mean.shape
        self.holding_cell -= np.reshape(self.x_mean, (256,1))
        sigma = np.cov(self.holding_cell)
           
        # sigma = np.zeros( ( row, row ) )
        
        # for i in range( 0, col):
        #     col_vector = self.holding_cell[ :, i ]
        #     col_vector = np.resize( col_vector, ( col_vector.size, 1 ) )
        #     col_vector = col_vector * col_vector.T
        #     col_vector *= ( 1 / col )
        #     sigma += col_vector
        
        return sigma
    
    def get_sub_images( self, image_array ):
        row_col = 16
        holding_cell = np.zeros( (row_col**2, 1) )
        
        for row in range(0, image_array.shape[0] - row_col, row_col):
            for col in range(0, image_array.shape[1] - row_col, row_col):
                current = np.reshape(image_array[row:row+row_col, col:col+row_col].flatten(), (row_col**2, 1))
                #print current.shape
                #print holding_cell.shape
                holding_cell = np.append(holding_cell, current, 1)
                
        self.holding_cell = np.delete(holding_cell, 0, 1)
        
        #self.holding_cell = np.zeros( (row_col**2, 1) )
        # current_array = np.zeros( (row_col**2, 1) )
        # start_row = 0
        # start_col = 0
        
        # while start_row < self.image_row - row_col:
        #     while start_col < self.image_col - row_col:
        #         current_row = start_row
        #         current_col = start_col
        #         position = 0
        #         for i in range ( current_row, current_row + row_col - 1 ):
        #             for j in range ( current_col, current_col + row_col - 1):
        #                 current_array[position] = image_array[ i ][ j ]
        #                 position += 1
                        
            
        #         start_col += row_col
        #         self.holding_cell = np.append(self.holding_cell, current_array, 1)
        #     start_row += row_col
        #     start_col = 0
            
        # self.row_count = m.floor( self.image_row / row_col  - 1)
        # self.col_count = m.floor( self.image_col / row_col - 1)
        # self.holding_cell = np.delete(self.holding_cell, 0, 1)
        # print(self.col_count)
        return self.holding_cell
                
                
    
    def get_x_approx(self, sigma):
        ( row, col ) = self.holding_cell.shape
        
        num_eigens = 256
        
        #compressed_cells = np.zeros( ( num_eigens, 1 ) )
      
        values, self.small_eigen = np.linalg.eig(sigma)
        self.small_eigen = self.small_eigen[:,0:num_eigens]
        compressed_cells = self.holding_cell.T.dot(self.small_eigen).T 
        
        # ( a, b ) = self.small_eigen.shape
        
        # for i in range ( 0 , b - num_eigens ):
        #     self.small_eigen = np.delete( self.small_eigen, num_eigens, 1)
                
        
        # for i in range( 0, col ):
        #     compressed_cells = np.append( compressed_cells, np.matrix( self.small_eigen.T ) * np.matrix( self.holding_cell[ :, i ] ).T, 1 )
        
        # compressed_cells = np.delete(compressed_cells, 0, 1)
        
        return compressed_cells
    
    def recover_x( self, compressed_stuff ):
        ( row, col ) = compressed_stuff.shape
        
        #reduce_to = 256
        
        reconstruction = compressed_stuff.T.dot(self.small_eigen.T).T + np.reshape(self.x_mean, (256,1))
        print "recon", reconstruction.shape

        final_image = np.empty(self.new_array.shape)
        row_col = 16
        index = 0
        for row in range(0, self.new_array.shape[0] - row_col, row_col):
            for col in range(0, self.new_array.shape[1] - row_col, row_col):
                final_image[row:row+row_col, col:col+row_col] = reconstruction[:,index].reshape((row_col,row_col))
                index += 1
        return final_image
        
        # for i in range ( 0, col - 1 ):
        #     holder_vector = compressed_stuff[ :, i ]
        #     holder_vector = np.matrix( self.small_eigen ) * np.matrix( holder_vector )
        #     for j in range (0 , row - 1):
        #         compressed_stuff[ j, i ] = holder_vector[ j ]
        
        # new_image = np.zeros( (  m.ceil( reduce_to ** ( 1 / 2 ) ) * self.row_count, m.ceil( reduce_to ** ( 1 / 2 ) ) * self.col_count ) )
        
        # ( a, b ) = new_image.shape
        # col_count = 0
        # row_count = 0
        # z = 0
        # for i in range( 0, col - 1):
        #     holding = compressed_stuff[ :, i ]
        #     while row_count < a:
        #         while col_count < b:
        #             for j in range( row_count, row_count + m.ceil( reduce_to ** ( 1 / 2 ) ) ):
        #                 for k in range ( col_count, col_count + m.ceil( reduce_to ** ( 1 / 2 ) ) ):
        #                     new_image[ j, k ] = holding[ z ]
        #                     z += 1
        #             z = 0
        #             col_count += m.ceil( reduce_to ** ( 1 / 2 ) )
        #         col_count = 0
        #         row_count += m.ceil( reduce_to ** ( 1 / 2 ) )
        
        # new_image_tri = np.zeros( ( a, b, 3 ) )
        # for i in range( 0, a ):
        #     for j in range( 0, b):
        #         for k in range (0, 3):
        #             new_image_tri[i][j][k] = new_image[i][j]
        
        # pyplot.imshow(new_image_tri)
        # return new_image_tri
            
    
  
                

test = pca( "imageOne.jpg" )
me = test.get_main_image()
you = test.format_main_image(me)
he = test.get_sub_images(you)
print he.shape
pyplot.imshow(he[:,0].reshape((16,16)))
pyplot.show()
she = pca.get_sigma_matrix(test)
print "eig shape", she.shape
they = pca.get_x_approx(test, she)
print "compressed shape", they.shape

we = pca.recover_x(test, they)
print np.min(we)
print np.max(we)
we = np.array(we, dtype='uint8')
final = np.empty((we.shape[0], we.shape[1], 3))
final[:,:,0] = we
final[:,:,1] = we
final[:,:,2] = we

pyplot.imshow(final)
print we
pyplot.show()
