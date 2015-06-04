
#Initialize Package: kinematics

def set_file_path( verbose=False ):  
    '''
    execute the set_file_path function of the super package. 
    -> an absolute path has to be set only at root level of packages.
    '''
    import os 
    import sys
    
    file_path   = os.path.abspath( os.path.dirname( __file__ ) )  
    super_path  = os.path.abspath( os.path.join( file_path, '../..' ))
    
    if verbose :
        print 
        print file_path
        print super_path
        print 
    
    sys.path.append(  super_path )
    
    # super package magiks: 
    import magiks
    return magiks.set_file_path( verbose=verbose ) 
