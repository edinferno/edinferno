/* 
* @File: definitions.h
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:47:59
* @Last Modified by:   ubuntubeast
* @Last Modified time: 2015-04-07 16:32:46
* @Desc: Debugging definitions
*/

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#define ERR(x) std::cerr << "\033[22;31;1m" << x << "\033[0m";  // RED
#define WARN(x) std::cerr << "\033[22;33;1m" << x << "\033[0m"; // YELLOW
#define INFO(x) std::cerr << "\033[22;37;1m" << x << "\033[0m"; // WHITE
#define DEBUG(x) std::cerr << "\033[22;34;1m" << x << "\033[0m";// BLUE
#define CLEAR() std::cerr << "\x1B[2J\x1B[H"; // CSI[2J clears screen, CSI[H moves the cursor to top-left corner

#endif /* DEFINITIONS_H_ */