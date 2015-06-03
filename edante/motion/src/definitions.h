/**
 * @file      definitions.h
 * @brief     Debugging definitions
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-19
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#define ERR(x) std::cerr << "\033[22;31;1m" << x << "\033[0m";    // RED
#define WARN(x) std::cerr << "\033[22;33;1m" << x << "\033[0m";   // YELLOW
#define INFO(x) std::cerr << "\033[22;37;1m" << x << "\033[0m";   // WHITE
#define DEBUG(x) std::cerr << "\033[22;34;1m" << x << "\033[0m";  // BLUE
#define CLEAR() std::cerr << "\x1B[2J\x1B[H";
// CSI[2J clears screen, CSI[H moves the cursor to top-left corner

#endif /* DEFINITIONS_H_ */
