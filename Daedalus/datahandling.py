from daedalus.smoother import Smoother

def main():

    window_length = 10  #length of rolling average window
    threshold = 100  # average at which metal is present


    while True:
        # import new data from arduino as new_data
        rolling_average = Smoother.process(Smoother(window_length), new_data)

        if rolling_average < threshold:
            print("metal")
            #set arduino red LED pin to high (D9)
        else:
            print("not metal")
            #set arduino green LED pin to high (D10)

if __name__ == "__main__":
    main()
