import math
import random
from string import ascii_uppercase

# Let X and Y be represented in meters.
# Let W be the normalized weight of the interest point.
# Let T represent the average time spent visiting the interest point.
class InterestPoint:
    def __init__(self, x=None, y=None, w=None, t=None, name=None, t_open=None, t_close=None):
        # self.x = None
        # self.y = None
        # self.w = None
        # self.t = None
        # self.t_open = None
        # self.t_close = None
        # self.name = None
        if x is not None:
            self.x = x
        else:
            self.x = int(random.random() * 4000)
        if y is not None:
            self.y = y
        else:
            self.y = int(random.random() * 4000)
        if w is not None:
            self.w = w
        else:
            self.w = int(random.random() * 100)
        if t is not None:
            self.t = t
        else:
            self.t = int(random.random() * 120)
        if t_open is not None:
            self.t_open = t_open
        else:
            self.t_open = 540
        if t_close is not None:
            self.t_close = t_close
        else:
            self.t_close = 1080
        if name is not None:
            self.name = name
        else:
            self.name = ''.join(random.choice(ascii_uppercase) for i in range(8))

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getW(self):
        return self.w

    def getT(self):
        return self.t

    def getName(self):
        return self.name

    def timeTo(self, point):
        xDistance = abs(self.getX() - point.getX())
        yDistance = abs(self.getY() - point.getY())
        time = (math.sqrt((xDistance * xDistance) + (yDistance * yDistance))/50)
        return time

    def __repr__(self):
        return str(self.getX()) + ", " + str(self.getY()) + ", " + str(self.getW()) + ", " + str(self.getT()) + ", " + str(self.getName())

class Bucket:
    timeUsed = 0
    start_time = None
    end_time = None
    totalWeight = 0

    def __init__(self, start_time, end_time):
        self.start_time = start_time
        self.end_time = end_time

    def getTime(self, index):
        return self.destinationPoints[index]

    def incrementTime(self, time):
        self.timeUsed += time

    def getWeight(self, index):
        return self.destinationPoints[index]

    def incrementWeight(self, weight):
        self.totalWeight += weight

class TourManager:

    base = None
    destinationPoints = []

    def __init__(self, origin):
        self.base = origin

    def addPoint(self, point):
        self.destinationPoints.append(point)

    def getPoint(self, index):
        return self.destinationPoints[index]

    def numberOfPoints(self):
        return len(self.destinationPoints)

    def allocateHeuristic(self):
        tourScore = 0
        buckets = []
        for i in range(0, 3):
            buckets.append(Bucket(540, 1080))
        removed = set()
        for bucket in buckets:
            if len(removed) >= len(self.destinationPoints):
                break
            previousPoint = self.base
            search = True
            while True:
                expectedReturns = self.calculateExpectedReturns(removed, previousPoint)
                if len(expectedReturns) == 0:
                    break
                for pointIndex in range(0, len(expectedReturns)):
                    currentPoint = expectedReturns[pointIndex][0]
                    if currentPoint.t + previousPoint.timeTo(currentPoint) + bucket.start_time + bucket.timeUsed <= bucket.end_time and bucket.start_time <= currentPoint.t_open <= bucket.end_time and currentPoint.t_close <= bucket.end_time:
                        bucket.incrementWeight(currentPoint.w)
                        bucket.incrementTime(currentPoint.t + previousPoint.timeTo(currentPoint))
                        previousPoint = currentPoint
                        removed.add(currentPoint.name)
                        break
                    if pointIndex == len(expectedReturns) - 1:
                        search = False
                if not search:
                    break
            tourScore += bucket.totalWeight
        return tourScore

    def expectedReturn(self, travelTime, activityTime, weight):
        return (activityTime/(travelTime + activityTime))*weight

    def calculateExpectedReturns(self, removed, previousPoint):
        returns = []
        for city in self.destinationPoints:
            if city.name in removed:
                continue
            else:
                travelTime = previousPoint.timeTo(city)
                returns.append((city, self.expectedReturn(travelTime, city.t, city.w)))
        return sorted(returns, key=lambda x: x[1], reverse=True)

class Tour:
    def __init__(self, tourmanager, tour=None):
        self.tourmanager = tourmanager
        self.tour = []
        self.fitness = 0.0
        self.score = 0
        self.base = tourmanager.base
        if tour is not None:
            self.tour = tour
        else:
            for i in range(0, self.tourmanager.numberOfPoints()):
                self.tour.append(None)

    def __len__(self):
        return len(self.tour)

    def __getitem__(self, index):
        return self.tour[index]

    def __setitem__(self, key, value):
        self.tour[key] = value

    def __repr__(self):
        geneString = "|"
        for i in range(0, self.tourSize()):
            geneString += str(self.getPoint(i)) + "|"
        return geneString

    def generateIndividual(self):
        for pointIndex in range(0, self.tourmanager.numberOfPoints()):
            self.setPoint(pointIndex, self.tourmanager.getPoint(pointIndex))
        random.shuffle(self.tour)

    def getPoint(self, tourPosition):
        return self.tour[tourPosition]

    def setPoint(self, tourPosition, point):
        self.tour[tourPosition] = point
        self.fitness = 0.0
        self.score = 0

    def getFitness(self):
        if self.fitness == 0:
            self.fitness = float(self.getScore())
        return self.fitness

    def getScore(self):
        if self.score == 0:
            tourScore = 0
            buckets = []
            for i in range(0, 2):
                buckets.append(Bucket(540, 1080))
            removed = set()
            for bucket in buckets:
                previousPoint = self.base
                for pointIndex in range(0, self.tourSize()):
                    currentPoint = self.getPoint(pointIndex)
                    if currentPoint.name in removed:
                        continue

                    if currentPoint.t + previousPoint.timeTo(currentPoint) + bucket.start_time + bucket.timeUsed <= bucket.end_time and bucket.start_time <= currentPoint.t_open <= bucket.end_time and currentPoint.t_close <= bucket.end_time:
                        bucket.incrementWeight(currentPoint.w)
                        bucket.incrementTime(currentPoint.t + previousPoint.timeTo(currentPoint))
                        previousPoint = currentPoint
                        removed.add(currentPoint.name)
                    else:
                        continue
                tourScore += bucket.totalWeight
            self.score = tourScore
        return self.score


    def tourSize(self):
        return len(self.tour)

    def containsPoint(self, point):
        return point in self.tour

class Population:
    def __init__(self, tourmanager, populationSize, initialise):
        self.tours = []
        for i in range(0, populationSize):
            self.tours.append(None)

        if initialise:
            for i in range(0, populationSize):
                newTour = Tour(tourmanager)
                newTour.generateIndividual()
                self.saveTour(i, newTour)

    def __setitem__(self, key, value):
        self.tours[key] = value

    def __getitem__(self, index):
        return self.tours[index]

    def saveTour(self, index, tour):
        self.tours[index] = tour

    def getTour(self, index):
        return self.tours[index]

    def getFittest(self):
        fittest = self.tours[0]
        for i in range(0, self.populationSize()):
            if fittest.getFitness() <= self.getTour(i).getFitness():
                fittest = self.getTour(i)
        return fittest

    def populationSize(self):
        return len(self.tours)

class GA:
    def __init__(self, tourmanager):
        self.tourmanager = tourmanager
        self.mutationRate = 0.1
        self.tournamentSize = 8
        self.elitism = True

    def evolvePopulation(self, pop):
        newPopulation = Population(self.tourmanager, pop.populationSize(), False)
        elitismOffset = 0
        if self.elitism:
            newPopulation.saveTour(0, pop.getFittest())
            elitismOffset = 1

        for i in range(elitismOffset, newPopulation.populationSize()):
            parent1 = self.tournamentSelection(pop)
            parent2 = self.tournamentSelection(pop)
            child = self.crossover(parent1, parent2)
            newPopulation.saveTour(i, child)

        for i in range(elitismOffset, newPopulation.populationSize()):
            self.mutate(newPopulation.getTour(i))

        return newPopulation

    def crossover(self, parent1, parent2):
        child = Tour(self.tourmanager)

        startPos = int(random.random() * parent1.tourSize())
        endPos = int(random.random() * parent1.tourSize())

        for i in range(0, child.tourSize()):
            if startPos < endPos and i > startPos and i < endPos:
                child.setPoint(i, parent1.getPoint(i))
            elif startPos > endPos:
                if not (i < startPos and i > endPos):
                    child.setPoint(i, parent1.getPoint(i))

        for i in range(0, parent2.tourSize()):
            if not child.containsPoint(parent2.getPoint(i)):
                for ii in range(0, child.tourSize()):
                    if child.getPoint(ii) == None:
                        child.setPoint(ii, parent2.getPoint(i))
                        break

        return child

    def mutate(self, tour):
        for tourPos1 in range(0, tour.tourSize()):
            if random.random() < self.mutationRate:
                tourPos2 = int(tour.tourSize() * random.random())

                point1 = tour.getPoint(tourPos1)
                point2 = tour.getPoint(tourPos2)

                tour.setPoint(tourPos2, point1)
                tour.setPoint(tourPos1, point2)

    def tournamentSelection(self, pop):
        tournament = Population(self.tourmanager, self.tournamentSize, False)
        for i in range(0, self.tournamentSize):
            randomId = int(random.random() * pop.populationSize())
            tournament.saveTour(i, pop.getTour(randomId))
        fittest = tournament.getFittest()
        return fittest

if __name__ == '__main__':
    tourmanager = TourManager(InterestPoint(2000, 2000, 0, 0, "BASE"))

    for i in range(25):
        point = InterestPoint()
        tourmanager.addPoint(point)
    heuristicScore = tourmanager.allocateHeuristic()
    print("Heuristic Weight: " + str(heuristicScore))

    for i in range(10):
        # Initialize population
        pop = Population(tourmanager, 160, True)
        print("GA (Run #" + str(i) + ") Initial Weight: " + str(pop.getFittest().getScore()))

        # Evolve population for 200 generations
        ga = GA(tourmanager)
        pop = ga.evolvePopulation(pop)
        for j in range(0, 200):
            pop = ga.evolvePopulation(pop)

        print("GA(Run #" + str(i) + ") Final Weight: " + str(pop.getFittest().getScore()))