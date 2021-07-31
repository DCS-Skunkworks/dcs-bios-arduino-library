#ifndef __DCSBIOS_NONDCSSTREAMLISTENER_H
#define __DCSBIOS_NONDCSSTREAMLISTENER_H

#include "Arduino.h"

namespace DcsBios {
	class NonDcsStreamListener {
		protected:
		public:
			virtual void onDcsBiosWrite(char value __attribute__((unused))) {}
			NonDcsStreamListener* nextNonDcsStreamListener;
			
			static NonDcsStreamListener* firstNonDcsStreamListener;
			NonDcsStreamListener() {
				// nothing in the list? insert self as first element.
				if (firstNonDcsStreamListener == NULL) {
					firstNonDcsStreamListener = this;
					nextNonDcsStreamListener = NULL;
					return;
				}
				
				// insert into list of export stream listeners,
				// keep list ordered ascending by lastAddressOfInterest, then ascending firstAddressOfInterest
				NonDcsStreamListener** prevNextPtr = &firstNonDcsStreamListener;
				NonDcsStreamListener* nextNDSL = firstNonDcsStreamListener;
				
				while (nextNDSL) {
					prevNextPtr = &(nextNDSL->nextNonDcsStreamListener);
					nextNDSL = nextNDSL->nextNonDcsStreamListener;
				}
				this->nextNonDcsStreamListener = nextNDSL;
				*prevNextPtr = this;
			}

			static void loopAll() {
				NonDcsStreamListener* ndsl = firstNonDcsStreamListener;
				while (ndsl) {
					ndsl->loop();
					ndsl = ndsl->nextNonDcsStreamListener;
				}
			}
			virtual void loop() {}
	};

	template < unsigned int LENGTH >
	class NonDcsStreamBuffer : public NonDcsStreamListener {
		private:
			char receivingBuffer[LENGTH+1];
			char userBuffer[LENGTH+1];
			volatile bool receivingDirty;
			bool userDirty;
			void (*callback)(char*);
			unsigned int rcvIdx;
		public:
			NonDcsStreamBuffer(void (*callback)(char*)) : NonDcsStreamListener() {
				memset(receivingBuffer, '\0', LENGTH);
				memset(userBuffer, '\0', LENGTH);
				
				receivingDirty = false;
				userDirty = false;
				this->callback = callback;
				rcvIdx = 0;
			}
			virtual void onDcsBiosWrite(char data) {
				if( data == '\n' && receivingDirty )
				{
					noInterrupts();
					memcpy(userBuffer, receivingBuffer, rcvIdx);
					userBuffer[rcvIdx] = '\0';
					receivingDirty = false;
					rcvIdx = 0;
					userDirty = true;
					interrupts();
				}
				else if( rcvIdx < LENGTH-1)
				{
					receivingBuffer[rcvIdx] = data;
					receivingDirty = true;
					rcvIdx++;
				}
			}
			virtual void onConsistentData() {
				// Will not get called; Parser won't know when the data is consistent
			}
			bool hasUpdatedData() {
				return userDirty;
			}
			char* getData() {
				userDirty = false;
				return userBuffer;
			}
			virtual void loop() {
				if (hasUpdatedData()) {
					if (callback) {
						callback(getData());
					}
				}
			}
	};
}

#endif
