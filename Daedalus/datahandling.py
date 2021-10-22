from Daedalus.utils.smoother import Smoother

def main():
    smoother = Smoother(20)

    with open('magnetic.csv') as f:
        data = f.readlines()
    data = [float(item.strip('\n')) for item in data]

    for d in data:
        print(Smoother.process(smoother,d))

if __name__ == "__main__":
    main()