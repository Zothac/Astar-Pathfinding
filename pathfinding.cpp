// title:   Astar Pathfinding
// date:    16.08.2020
// autor:   Daniel Nobis

#include <SFML/Graphics.hpp>
#include <iostream>
#include <sstream>

int RES_X = 1024;           // resolution x of main window
int RES_Y = 768;            // resolution y of main window

const int MAP_WIDTH = 15;   // map width
const int MAP_HEIGTH = 10;  // map heigth

int CURSOR_X = 0;           // position x of cursor within map[MAP_WIDTH][MAP_HEIGTH] coordinate system
int CURSOR_Y = 0;           // position y of cursor within map[MAP_WIDTH][MAP_HEIGTH] coordinate system
int CURSOR_TIMER = 0;       // refresh timer for user input

int SHOW_NODE_COSTS = 0;    // show node costs
int SHOW_HELP = 0;          // show help
int SHOW_NODES = 1;         // show nodes

int TILE_SIZE = 64;         // absolute size of each tile in pixel

int CAMERA_X = RES_X / 2 - (MAP_WIDTH * TILE_SIZE) / 2;     // offset x for all graphics
int CAMERA_Y = RES_Y / 2 - (MAP_HEIGTH * TILE_SIZE) / 2;    // offset y for all graphics

int START_X = 2;            // start position x of pathfinding
int START_Y = 5;            // start position y of pathfinding

int TARGET_X = 12;          // target position x of pathfinding
int TARGET_Y = 5;           // target position y of pathfinding

const int MAX_NODES = 5000;         // max allowed number of nodes
int NODE_X[MAX_NODES] = {};         // X coordinate of node
int NODE_Y[MAX_NODES] = {};         // Y coordinate of node
int NODE_PARENT[MAX_NODES] = {};    // index number of parent node
int NODE_F[MAX_NODES] = {};         // F cost
int NODE_G[MAX_NODES] = {};         // G cost
int NODE_H[MAX_NODES] = {};         // h cost
int NODE_ISCLOSED[MAX_NODES] = {};  // defines status of node (0 = node is open, 1 = node is closed)
int NODE_COUNT = 0;                 // counts each node

int NODE_MAP[MAP_HEIGTH][MAP_WIDTH] = {};   // array with all closed nodes

const int MAX_PATH = 5000;          // max path length
int PATH_X[MAX_PATH] = {};          // X coordinate of path entry
int PATH_Y[MAX_PATH] = {};          // y coordinate of path entry
int PATH_COUNT = 0;                 // counts each path entry
bool PATH_FOUND = 0;                // 1 = path found, 0 = no path found

int PATH_SEARCH_DIRECTIONS = 1;     // 0 = eight, 1 = eight(no cutting), 2 = four, 3 = four (diagonal)
int PATH_SEARCH_HEURISTIC = 1;      // 0 = none,  1 = manhatten, 2 = diagonal shortcut, 3 = own method


int MAP[MAP_HEIGTH][MAP_WIDTH] = {  // 0 - field is empty, nodes could be created, 1 - field is blocked for nodes
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};


void CREATE_NODE(int X,int Y,int G, int PARENT_ID, int NODE_MAP[MAP_HEIGTH][MAP_WIDTH], int MAP[MAP_HEIGTH][MAP_WIDTH]) {   // creates new open node
    if (X < 0 || X > MAP_WIDTH-1) { return; }   // out of map
    if (Y < 0 || Y > MAP_HEIGTH-1) { return; }  // out of map
    
    if (NODE_MAP[Y][X] == 1) { return; } // closed node already exists here
    if (MAP[Y][X] == 1) { return; }      // field blocked
    

    // prüfen ob hier bereits eine open-node vorhanden ist   
    for (int i = 0; i < NODE_COUNT; i++) {          // alle knoten prüfen
        if(NODE_X[i] == X && NODE_Y[i] == Y){       // wenn hier bereits ein knoten vorhanden ist
            if (NODE_G[i] > G) {                    // falls neuer knoten besser wäre, alten überschreiben:
                NODE_PARENT[i] = PARENT_ID;             // abhängigkeit anpassen
                NODE_G[i] = G;                          // wegkosten vom startpunkt neu berechnen
                NODE_F[i] = NODE_G[i] + NODE_H[i];      // gesamtwegkosten neu berechnen
                NODE_MAP[NODE_Y[i]][NODE_X[i]] = 1;     // put node into map of closed nodes
                NODE_ISCLOSED[i] = 1;                   // define node as closed node
                return;
            }
            else    // neuer Knoten wäre nicht besser
            {
            return; // nichts machen
            }

        }
     
    }

    // wenn kein knoten angepasst wurde, neuen knoten erstellen
    NODE_X[NODE_COUNT] = X;
    NODE_Y[NODE_COUNT] = Y;
    NODE_PARENT[NODE_COUNT] = PARENT_ID;
    
    // calculate heuristic
    int CURRENT_HEURISTIC = 0;
    int X_DISTANCE = 0;
    int Y_DISTANCE = 0;
    switch (PATH_SEARCH_HEURISTIC) {                                                    // select heuristic
        case 0: CURRENT_HEURISTIC = 0;                                                  //  0 - no heuristic
                break;
        case 1: CURRENT_HEURISTIC = 10 * abs(X - TARGET_X) + abs(Y - TARGET_Y);         //  1 - manhatten
                break;
        case 2: X_DISTANCE = abs(X - TARGET_X);                                         //  2 - diagonal shortcut 
                Y_DISTANCE = abs(Y - TARGET_Y);
                if (X_DISTANCE > Y_DISTANCE) {CURRENT_HEURISTIC = 14 * Y_DISTANCE + 10 * (X_DISTANCE - Y_DISTANCE);}
                else {CURRENT_HEURISTIC = 14 * Y_DISTANCE + 10 * (Y_DISTANCE - X_DISTANCE);}
                break;
        case 3: CURRENT_HEURISTIC = 0;                                                  //  3 - own method
                if (abs(X - TARGET_X) > abs(Y - TARGET_Y)) { CURRENT_HEURISTIC = abs(X - TARGET_X); }
                if (abs(X - TARGET_X) < abs(Y - TARGET_Y)) { CURRENT_HEURISTIC = abs(Y - TARGET_Y); }
                if (CURRENT_HEURISTIC == 0) { CURRENT_HEURISTIC = abs(X-TARGET_X); }
                CURRENT_HEURISTIC = CURRENT_HEURISTIC * 10;
                break;
    }

    NODE_G[NODE_COUNT] = G;                                             // waycosts from start (G)
    NODE_H[NODE_COUNT] = CURRENT_HEURISTIC;                             // waycosts to target  (H)
    NODE_F[NODE_COUNT] = NODE_G[NODE_COUNT] + NODE_H[NODE_COUNT];       // calculate waycosts  (F)

    NODE_ISCLOSED[NODE_COUNT] = 0;  // new node is open-node 

    NODE_COUNT++;                   // count this node
}

void CREATE_PATH() {
    // this function is called from SEARCH_PATCH() after a valid path has been found
    // the target position is known and a path will be generated from this position to the start position
    
    PATH_COUNT = 0; // select first path entry
    int ID = 0;     // ID number of nodes

    // select node wich is on target and set this coordinates to the first path entry
    for (int i = 0; i < NODE_COUNT; i++) {
        if (NODE_X[i] == TARGET_X && NODE_Y[i] == TARGET_Y) {
            PATH_X[PATH_COUNT] = NODE_X[i];
            PATH_Y[PATH_COUNT] = NODE_Y[i];
            PATH_COUNT++;
            ID = i;
        }
    }
           
    bool SEEK = true;       // enable seek

    while (SEEK == true) {  // while seek is enabled (loop begins)

        if (PATH_COUNT > MAX_PATH) { std::cout << "Pathfinding-Error: MAX PATHLENGTH REACHED!"; SEEK = false; return; }    // if max pathlength is reached, abort search

        int TMP_ID = ID;
        ID = NODE_PARENT[TMP_ID];   // select the parent node of the current node (search backwards)

        PATH_X[PATH_COUNT] = NODE_X[ID];    // make path entry
        PATH_Y[PATH_COUNT] = NODE_Y[ID];    // make path entry
 
        if (START_X == PATH_X[PATH_COUNT] && START_Y == PATH_Y[PATH_COUNT]) {  // if current path entry = start position - search is over
            PATH_COUNT++;
            PATH_X[PATH_COUNT] = NODE_X[ID];
            PATH_Y[PATH_COUNT] = NODE_Y[ID];
            SEEK = false; 
            return;
        }

        PATH_COUNT++;   // count path entry
    }
         
    return; // no valid path found here
}

bool FIND_PATH() {

    // delete old path
    PATH_COUNT = 0;
    int ID = 0;

    for (int i = 0; i < MAX_PATH; i++) {
        PATH_X[i] = 0;
        PATH_Y[i] = 0;
    }

    // delete every node
    for (int i = 0; i < MAX_NODES; i++) {
        NODE_X[i] = 0;
        NODE_Y[i] = 0;
        NODE_PARENT[i] = 0;
        NODE_F[i] = 0;
        NODE_G[i] = 0;
        NODE_H[i] = 0;
        NODE_ISCLOSED[i] = 0;
    }
    NODE_COUNT = 0;

    // delete NODE_MAP[][]
    for (int y = 0; y < MAP_HEIGTH; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            NODE_MAP[y][x] = 0;
        }
    }

    CREATE_NODE(START_X, START_Y, 0, 0, NODE_MAP, MAP); // first node at start position

    bool SEEK_PATH = true;      // enable pathfinding

    while (SEEK_PATH == true) { // while pathfinding is enabled (loop begins)

        // check if target is found
        int CLOSE_COUNT = 0;
        for (int i = 0; i < NODE_COUNT; i++) {
          if (NODE_ISCLOSED[i] == 1) { CLOSE_COUNT++; }                                                   // geschlossene knoten zählen
          if (CLOSE_COUNT == NODE_COUNT) { PATH_FOUND = 0; return 0; }                                    // wenn alle knoten geschlossen, kein ziel gefunden
          if (NODE_X[i] == TARGET_X && NODE_Y[i] == TARGET_Y) {                                           // if target is found 
              SEEK_PATH = false;    // end pathfinding
              CREATE_PATH();        // create path list backwards from target to start
              PATH_FOUND = 1;       // status for external use
              return 1;
          }
        }

        // seek open-node with the lowest F cost
        int MIN_F_COST = 15000000;
        int MIN_F_ID = 0;
        for (int i = 0; i < NODE_COUNT; i++) {
            if (NODE_ISCLOSED[i] == 0) { if (NODE_F[i] < MIN_F_COST) { MIN_F_COST = NODE_F[i]; MIN_F_ID = i; } }
        }

        // select open-node with lowest F cost 
        int X = NODE_X[MIN_F_ID];
        int Y = NODE_Y[MIN_F_ID];
        int ID = MIN_F_ID;
        
        NODE_ISCLOSED[ID] = 1; // this node is closed now
        NODE_MAP[Y][X] = 1;    // put this node to the closed node map

        // create new nodes around current node
        switch(PATH_SEARCH_DIRECTIONS) {                                        // select allowed search directions

            case 0: {   // eight
                CREATE_NODE(X, Y - 1, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // above      
                CREATE_NODE(X + 1, Y - 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP);  // right above
                CREATE_NODE(X + 1, Y, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // right
                CREATE_NODE(X + 1, Y + 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP);  // right under
                CREATE_NODE(X, Y + 1, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // under
                CREATE_NODE(X - 1, Y + 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP);  // left under
                CREATE_NODE(X - 1, Y, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // left
                CREATE_NODE(X - 1, Y - 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP);  // left above
                break;
            }

            case 1: { // eight (no cutting)
                CREATE_NODE(X, Y - 1, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // above      
                if (MAP[Y][X + 1] == 0 && MAP[Y - 1][X] == 0) { CREATE_NODE(X + 1, Y - 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP); }  // right above
                CREATE_NODE(X + 1, Y, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // right
                if (MAP[Y][X + 1] == 0 && MAP[Y + 1][X] == 0) { CREATE_NODE(X + 1, Y + 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP); }  // right under
                CREATE_NODE(X, Y + 1, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // under
                if (MAP[Y][X - 1] == 0 && MAP[Y + 1][X] == 0) { CREATE_NODE(X - 1, Y + 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP); }  // left under
                CREATE_NODE(X - 1, Y, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // left
                if (MAP[Y][X - 1] == 0 && MAP[Y - 1][X] == 0) { CREATE_NODE(X - 1, Y - 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP); }  // left above
                break;
            }

            case 2: {  // four
                CREATE_NODE(X, Y - 1, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // above 
                CREATE_NODE(X + 1, Y, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // right
                CREATE_NODE(X, Y + 1, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // under
                CREATE_NODE(X - 1, Y, NODE_G[ID] + 10, ID, NODE_MAP, MAP);      // left
                break;
            }

            case 3: {  // four diagonal
                CREATE_NODE(X + 1, Y - 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP);  // right above
                CREATE_NODE(X + 1, Y + 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP);  // right under
                CREATE_NODE(X - 1, Y + 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP);  // left under
                CREATE_NODE(X - 1, Y - 1, NODE_G[ID] + 14, ID, NODE_MAP, MAP);  // left above
                break;
            }
        }
      
    } // repeat ... while (SEEK_PATH == true)
    

}

void RESET_ALL(int MAP[MAP_HEIGTH][MAP_WIDTH], int NODE_MAP[MAP_HEIGTH][MAP_WIDTH]) {
    
    // delete old path
    PATH_COUNT = 0;
    int ID = 0;
    for (int i = 0; i < MAX_PATH; i++) {
        PATH_X[i] = 0;
        PATH_Y[i] = 0;
    }

    // delete every node
    for (int i = 0; i < MAX_NODES; i++) {
        NODE_X[i] = 0;
        NODE_Y[i] = 0;
        NODE_PARENT[i] = 0;
        NODE_F[i] = 0;
        NODE_G[i] = 0;
        NODE_H[i] = 0;
        NODE_ISCLOSED[i] = 0;
    }
    NODE_COUNT = 0;

    // delete NODE_MAP[][]
    for (int y = 0; y < MAP_HEIGTH; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            NODE_MAP[y][x] = 0;
            MAP[y][x] = 0;
        }
    }

    START_X = 1;
    START_Y = MAP_HEIGTH/2;

    TARGET_X = MAP_WIDTH-2;
    TARGET_Y = MAP_HEIGTH/2;
}

void EDIT_UPDATE_INPUT(int map[MAP_HEIGTH][MAP_WIDTH]) {
    // --- ArrowKeys move cursor
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))    { CURSOR_Y = CURSOR_Y - 1; }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))  { CURSOR_Y = CURSOR_Y + 1; }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))  { CURSOR_X = CURSOR_X - 1; }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) { CURSOR_X = CURSOR_X + 1; }

    if (CURSOR_Y < 0) { CURSOR_Y = 0; }
    if (CURSOR_Y > MAP_HEIGTH - 1) { CURSOR_Y = MAP_HEIGTH - 1; }
    if (CURSOR_X < 0) { CURSOR_X = 0; }
    if (CURSOR_X > MAP_WIDTH - 1) { CURSOR_X = MAP_WIDTH - 1; }
    
    // --- set walls
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
        if (START_X != CURSOR_X || START_Y != CURSOR_Y) {        // don't set a wall at start position
            if (TARGET_X != CURSOR_X || TARGET_Y != CURSOR_Y) {  // don't set a wall at target position
                map[CURSOR_Y][CURSOR_X] = 1; FIND_PATH();        // set wall and build new path
            }
        }
    }
    
    // delete walls
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Delete)) { map[CURSOR_Y][CURSOR_X] = 0; FIND_PATH();}

    // --- show / hide help
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::F1)) {
        switch (SHOW_HELP) {
            case 0: SHOW_HELP = 1; break;
            case 1: SHOW_HELP = 0; break;
        }
    }

    // --- show waycosts of each node
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::C)) {
        switch (SHOW_NODE_COSTS) {
            case 0: SHOW_NODE_COSTS = 1; break;
            case 1: SHOW_NODE_COSTS = 0; break;
        }
    }

    // --- reset map
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::R)) { RESET_ALL(MAP, NODE_MAP); FIND_PATH(); }

    // --- change direction mode
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
        PATH_SEARCH_DIRECTIONS++;
        if (PATH_SEARCH_DIRECTIONS == 4) { PATH_SEARCH_DIRECTIONS = 0; }
        FIND_PATH();
    }

    // --- change the way to calculate h costs (heuristic)
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::H)) {
        PATH_SEARCH_HEURISTIC++;
        if (PATH_SEARCH_HEURISTIC == 4) { PATH_SEARCH_HEURISTIC = 0; }
        FIND_PATH();
    }

    // --- show / hide nodes
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::N)) {
        switch (SHOW_NODES) {
        case 0: SHOW_NODES = 1; break;
        case 1: SHOW_NODES = 0; break;
        }
    }

    // --- set start / target position
    if (map[CURSOR_Y][CURSOR_X] == 0) {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) { START_X = CURSOR_X; START_Y = CURSOR_Y; FIND_PATH(); }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::T)) { TARGET_X = CURSOR_X; TARGET_Y = CURSOR_Y; FIND_PATH(); }
    }

}

int main()
{
    FIND_PATH(); // find path

    std::ostringstream ss;  // string stream for text output

    sf::Font font1;
    font1.loadFromFile("arial.ttf");

    sf::Text txtgfx("", font1, 15);
    txtgfx.setFillColor(sf::Color(255, 255, 255));

    sf::RectangleShape framegfx(sf::Vector2f(TILE_SIZE, TILE_SIZE));    // background tile
    framegfx.setFillColor(sf::Color::Green);

    sf::Clock myclock;  // for timer
    myclock.restart();

    sf::RenderWindow window(sf::VideoMode(RES_X, RES_Y), "Astar pathfinding", sf::Style::Default);
    
    // --- main loop begins ---
    while (window.isOpen())
    {

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // --- check for user input
        if (myclock.getElapsedTime().asMilliseconds() >= CURSOR_TIMER + 150) {
            CURSOR_TIMER = myclock.getElapsedTime().asMilliseconds();
            EDIT_UPDATE_INPUT(MAP);
        }

        // --- graphic begin ---
        window.clear();

        // --- draw map ---
        framegfx.setOutlineThickness(1);
        framegfx.setOutlineColor(sf::Color(128, 128, 128));
        for (int y = 0; y < MAP_HEIGTH; y++) {
            for (int x = 0; x < MAP_WIDTH; x++) {
                framegfx.setPosition(x * TILE_SIZE + CAMERA_X, y * TILE_SIZE + CAMERA_Y);
                framegfx.setFillColor(sf::Color(0, 0, 0, 255));
                if (MAP[y][x] == 1) { framegfx.setFillColor(sf::Color(196, 196, 196)); };
                window.draw(framegfx);
            }
        }

        // --- draw path ---
        for (int i = 0; i < PATH_COUNT; i++) {
            framegfx.setPosition(PATH_X[i] * TILE_SIZE + CAMERA_X, PATH_Y[i] * TILE_SIZE + CAMERA_Y);
            framegfx.setFillColor(sf::Color(128, 0, 128, 128));
            window.draw(framegfx);
        }

        // --- draw cursor ---
        framegfx.setPosition(CURSOR_X * TILE_SIZE + CAMERA_X, CURSOR_Y * TILE_SIZE + CAMERA_Y);
        framegfx.setFillColor(sf::Color(0, 0, 0, 0));
        framegfx.setOutlineColor(sf::Color(255, 255, 0));
        framegfx.setOutlineThickness(1);
        window.draw(framegfx);

        // --- draw start position ---
        framegfx.setPosition(START_X * TILE_SIZE + CAMERA_X, START_Y * TILE_SIZE + CAMERA_Y);
        framegfx.setFillColor(sf::Color(0, 255, 0, 255));
        framegfx.setOutlineThickness(0);
        window.draw(framegfx);

        // --- draw target position ---
        framegfx.setPosition(TARGET_X * TILE_SIZE + CAMERA_X, TARGET_Y * TILE_SIZE + CAMERA_Y);
        framegfx.setFillColor(sf::Color(255, 0, 0, 255));
        framegfx.setOutlineThickness(0);
        window.draw(framegfx);

        // --- draw nodes costs begin ---
        if (SHOW_NODE_COSTS == 1) {
            for (int i = 0; i < NODE_COUNT; i++) {

                // show H cost
                ss.str("");
                ss << NODE_F[i];
                txtgfx.setFillColor(sf::Color(255, 255, 255));
                txtgfx.setPosition(5 + NODE_X[i] * TILE_SIZE + CAMERA_X, NODE_Y[i] * TILE_SIZE + CAMERA_Y);
                txtgfx.setString(ss.str());
                window.draw(txtgfx);

                // show G cost
                ss.str("");
                ss << NODE_G[i];
                txtgfx.setFillColor(sf::Color(255, 255, 255));
                txtgfx.setPosition(5 + NODE_X[i] * TILE_SIZE + CAMERA_X, 45 + NODE_Y[i] * TILE_SIZE + CAMERA_Y);
                txtgfx.setString(ss.str());
                window.draw(txtgfx);

                // show H cost
                ss.str("");
                ss << NODE_H[i];
                txtgfx.setFillColor(sf::Color(255, 255, 255));
                txtgfx.setPosition(40 + NODE_X[i] * TILE_SIZE + CAMERA_X, 45 + NODE_Y[i] * TILE_SIZE + CAMERA_Y);
                txtgfx.setString(ss.str());
                window.draw(txtgfx);
            }
        }
        // --- draw nodes costs end---


        // --- show help begin ---
        if (SHOW_HELP == 1) {
            std::ostringstream ss;
            ss << "S - set start    T - set target                    W - set wall                       Del - delete wall      ArrowKeys - move cursor";
            txtgfx.setFillColor(sf::Color(255, 255, 255));
            txtgfx.setPosition(10, 30);
            txtgfx.setString(ss.str());
            window.draw(txtgfx);

            ss.str("");
            ss << "F1 - Help          C - show/hide costs          N - Show/hide nodes          R - Reset all          D - change search directions          H - change heuristic";
            txtgfx.setFillColor(sf::Color(255, 255, 255));
            txtgfx.setPosition(10, 5);
            txtgfx.setString(ss.str());
            window.draw(txtgfx);
            // --- show help end ---
        }
        else {
            ss.str("");
            ss << "F1 - Help";
            txtgfx.setFillColor(sf::Color(255, 255, 255));
            txtgfx.setPosition(10, 5);
            txtgfx.setString(ss.str());
            window.draw(txtgfx);
        }

        // --- show some stats and values begin ---
        ss.str("");
        if(PATH_FOUND) { ss << "valid path found"; }
        else { ss << "NO PATH FOUND"; }
        txtgfx.setFillColor(sf::Color(255, 255, 255));
        txtgfx.setPosition(10, RES_Y-56);
        txtgfx.setString(ss.str());
        window.draw(txtgfx);

        // --- show search directions
        ss.str("");
        ss << "Search direction: ";
        switch (PATH_SEARCH_DIRECTIONS) {
            case 0: ss << "eight"; break;
            case 1: ss << "eight (no cutting)"; break;
            case 2: ss << "four"; break;
            case 3: ss << "four (diagonal)"; break;
        }

        txtgfx.setFillColor(sf::Color(255, 255, 255));
        txtgfx.setPosition(10, RES_Y - 30);
        txtgfx.setString(ss.str());
        window.draw(txtgfx);

        // --- show pathlength
        ss.str("");
        ss << "pathlength: ";
        ss << PATH_COUNT;
        ss << " build with ";
        ss << NODE_COUNT;
        ss << " nodes";
       
        txtgfx.setFillColor(sf::Color(255, 255, 255));
        txtgfx.setPosition(400, RES_Y - 56);
        txtgfx.setString(ss.str());
        window.draw(txtgfx);

        // --- show heuristic
        ss.str("");
        ss << "heuristic: ";
        switch (PATH_SEARCH_HEURISTIC) {
            case 0: ss << "none"; break;
            case 1: ss << "manhatten"; break;
            case 2: ss << "diagonal shortcut"; break;
            case 3: ss << "own method"; break;
        }

        txtgfx.setFillColor(sf::Color(255, 255, 255));
        txtgfx.setPosition(400, RES_Y - 30);
        txtgfx.setString(ss.str());
        window.draw(txtgfx);

        // --- show some stats and values end ---

        // --- show nodes
        if (SHOW_NODES == 1) {
            for (int i = 0; i < NODE_COUNT; i++) {
                sf::Vertex line[] =
                {
                    sf::Vertex(sf::Vector2f((TILE_SIZE / 2) + CAMERA_X + NODE_X[i] * TILE_SIZE, (TILE_SIZE / 2) + CAMERA_Y + NODE_Y[i] * TILE_SIZE)),
                    sf::Vertex(sf::Vector2f((TILE_SIZE / 2) + CAMERA_X + NODE_X[NODE_PARENT[i]] * TILE_SIZE, (TILE_SIZE / 2) + CAMERA_Y + NODE_Y[NODE_PARENT[i]] * TILE_SIZE))
                };

                if (NODE_ISCLOSED[i]) {
                    line[0].color = sf::Color(0, 0, 196);
                    line[1].color = sf::Color(0, 0, 196);
                }
                else {
                    line[0].color = sf::Color(196, 0, 0);
                    line[1].color = sf::Color(196, 0, 0);
                }

                window.draw(line, 2, sf::Lines);
            }
        }

        // --- show path with yellow line
        for (int i = 1; i < PATH_COUNT; i++) {
            sf::Vertex line[] =
            {
                sf::Vertex(sf::Vector2f((TILE_SIZE / 2) + CAMERA_X + PATH_X[i] * TILE_SIZE, (TILE_SIZE / 2) + CAMERA_Y + PATH_Y[i] * TILE_SIZE)),
                sf::Vertex(sf::Vector2f((TILE_SIZE / 2) + CAMERA_X + PATH_X[i-1] * TILE_SIZE, (TILE_SIZE / 2) + CAMERA_Y + PATH_Y[i-1] * TILE_SIZE))
            };

            line[0].color = sf::Color(196, 196, 0);
            line[1].color = sf::Color(196, 196, 0);
            window.draw(line, 2, sf::Lines);
        }

        window.display();
        // --- graphic end ---

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) { return 10; }    // ESCAPE - quit programm

    }
    // --- main loop ends

    return 0;
}