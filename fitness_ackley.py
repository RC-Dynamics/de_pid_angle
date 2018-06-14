import numpy as np

def fitness_ackley(genotype = [0.0]):
    """
    fitness_ackley:
        Calculates the fitness of a genotype concerning Ackley function
    
    Keyword arguments
    _________________
    
        genotype (double vector) :
            Vector of genes (default: [0.0])

    Return
    ______

        The fitness of the genotype (double)
        
    """
    c1, c2 ,c3 = 20.0, 0.2, np.pi*2
    numGenes = np.int16(len(genotype))
    genotype = np.array(genotype)
    sum1 = np.sum( genotype**2 )
    sum2 = np.sum( np.cos(c3 * genotype ))
    
    term1 = (-c1) * np.exp( -c2 * (((1 / numGenes) * sum1) ** 0.5 ))
    term2 = -np.exp(( 1 / numGenes ) * sum2)
    err = term1 + c1 + term2 + 2.718281828459045
    
    return (err)


if __name__ == '__main__':
	genotype = [0.0, 0.0, 0.0]
	print (fitness_ackley(genotype))
