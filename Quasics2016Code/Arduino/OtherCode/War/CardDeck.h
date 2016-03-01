#ifndef CARDDECK_H_
#define CARDDECK_H_

#include "Arduino.h"

class CardDeck {
  public:
    CardDeck(long seed);
    
    void GetCard (unsigned char& Value, unsigned char& Suit);
    void Shuffle (long seed);
    void Reset ();
    
    int BeenDealt (unsigned char& Value, unsigned char& Suit);
    long GetSeed ();
  protected:
    bool aceSpades;
    bool twoSpades;
    bool threeSpades;
    bool fourSpades;
    bool fiveSpades;
    bool sixSpades;
    bool sevenSpades;
    bool eightSpades;
    bool nineSpades;
    bool tenSpades;
    bool jackSpades;
    bool queenSpades;
    bool kingSpades;
    
    bool aceHearts;
    bool twoHearts;
    bool threeHearts;
    bool fourHearts;
    bool fiveHearts;
    bool sixHearts;
    bool sevenHearts;
    bool eightHearts;
    bool nineHearts;
    bool tenHearts;
    bool jackHearts;
    bool queenHearts;
    bool kingHearts;
    
    bool aceClubs;
    bool twoClubs;
    bool threeClubs;
    bool fourClubs;
    bool fiveClubs;
    bool sixClubs;
    bool sevenClubs;
    bool eightClubs;
    bool nineClubs;
    bool tenClubs;
    bool jackClubs;
    bool queenClubs;
    bool kingClubs;
    
    bool aceDiamonds;
    bool twoDiamonds;
    bool threeDiamonds;
    bool fourDiamonds;
    bool fiveDiamonds;
    bool sixDiamonds;
    bool sevenDiamonds;
    bool eightDiamonds;
    bool nineDiamonds;
    bool tenDiamonds;
    bool jackDiamonds;
    bool queenDiamonds;
    bool kingDiamonds;
};


#endif
