#include "al_ext/soundfile/al_SoundfileBuffered.hpp"

#include <algorithm>
#include <iostream>

using namespace al;

SoundFileBuffered::SoundFileBuffered(int bufferFrames)
    : mSeek(-1), mCurPos(0), mBufferFrames(bufferFrames) {}

SoundFileBuffered::SoundFileBuffered(std::string fullPath, bool loop,
                                     int bufferFrames)
    : SoundFileBuffered(bufferFrames) {
  mLoop = loop;
  open(fullPath);
}

SoundFileBuffered::~SoundFileBuffered() { close(); }

bool SoundFileBuffered::open(std::string fullPath) {
  auto previousChannels = mSf.channels();
  if (fullPath != mSf.path()) {
    close();
    mSf.path(fullPath);
    mSf.openRead();
    if (mSf.opened()) {
      //      std::cout << "buffer frames (per channel): " << mBufferFrames
      //                << " channels " << channels() << std::endl;
      if (previousChannels != mSf.channels()) {
        if (mRingBuffer) {
          delete mRingBuffer;
        }
        //        if (mFileBuffer) {
        //          delete[] mFileBuffer;
        //        }
        mRingBuffer =
            new SingleRWRingBuffer(mBufferFrames * channels() * sizeof(float));
        mFileBuffer = new float[mBufferFrames * channels()];
      }
      if (!mRingBuffer) {
        mRingBuffer =
            new SingleRWRingBuffer(mBufferFrames * channels() * sizeof(float));
      } else {
        mRingBuffer->clear();
      }
      if (!mFileBuffer) {
        mFileBuffer = new float[mBufferFrames * channels()];
      }
      // TODO don't reallocate thread every time
      mReaderThread = new std::thread(readFunction, this);
      mRunning = true;
      mCurPos.store(0);
      // FIXME there is a race condition if open is called while read is
      // processing.

      // We need to make sure that the condition variable is already waiting
      // when waking up to pre-fill buffer.
      while (mRingBuffer->readSpace() <
             mBufferFrames * channels() * sizeof(float) - 8) {
        mCondVar.notify_one();
      }

      return true;
    }
  } else {
    // TODO don't reallocate thread every time
    mReaderThread = new std::thread(readFunction, this);
    mSeek.store(0);
    if (mRingBuffer) {
      mRingBuffer->clear();
    } else {
      mRingBuffer =
          new SingleRWRingBuffer(mBufferFrames * channels() * sizeof(float));
    }

    if (!mFileBuffer) {
      mFileBuffer = new float[mBufferFrames * channels()];
    }
    mRunning = true;
    mCurPos.store(0);
    while (mRingBuffer->readSpace() <
           mBufferFrames * channels() * sizeof(float) - 8) {
      mCondVar.notify_one();
    }
    return true;
  }
  return false;
}

bool SoundFileBuffered::close() {
  if (mSf.opened()) {
    mRunning = false;
    mCondVar.notify_one();
    mReaderThread->join();
    //    delete mReaderThread;
    //    delete mRingBuffer;
    //    delete[] mFileBuffer;
    //    mReaderThread = nullptr;
    //    mRingBuffer = nullptr;
    //    mFileBuffer = nullptr;
    mSf.close();
  }
  return true;
}

size_t SoundFileBuffered::read(float *buffer, int numFrames) {
  if (mRingBuffer) {
    size_t bytesRead = mRingBuffer->read(
        (char *)buffer, numFrames * channels() * sizeof(float));
    if (bytesRead != numFrames * channels() * sizeof(float)) {
      //    std::cerr << "Warning: underrun" << std::endl;
      // TODO: handle underrun
    }
    mCondVar.notify_one();
    return bytesRead / (channels() * sizeof(float));
  } else {
    return 0;
  }
}

bool SoundFileBuffered::opened() const { return mSf.opened(); }

void SoundFileBuffered::readFunction(SoundFileBuffered *obj) {
  while (obj->mRunning) {
    std::unique_lock<std::mutex> lk(obj->mLock);
    obj->mCondVar.wait(lk);
    int seek = obj->mSeek.load();
    if (seek >= 0) { // Process seek request
      obj->mSf.seek(seek, SEEK_SET);
      obj->mSeek.store(-1);
      obj->mCurPos = seek;
    }
    size_t framesToRead =
        obj->mRingBuffer->writeSpace() / (obj->channels() * sizeof(float));
    framesToRead = std::min(framesToRead, obj->mBufferFrames);
    if (framesToRead < obj->mBufferFrames - 1) {
      // The ring buffer is 1 sample smaller than allocated, so this check
      // is fudged to compensate for that.
      // TODO handle overrun
      //      std::cerr << "Warning: overrun available " << framesToRead << "
      //      needed "
      //                << obj->mBufferFrames << std::endl;
      //      continue;
    }
    int framesRead = obj->mSf.read<float>(obj->mFileBuffer, framesToRead);
    std::atomic_fetch_add(&(obj->mCurPos), framesRead);
    if ((size_t)framesRead !=
        framesToRead) { // Final incomplete buffer in the file
      if (obj->mLoop) {
        obj->mSf.seek(0, SEEK_SET);
        std::atomic_fetch_add(&(obj->mRepeats), 1);
        framesRead += obj->mSf.read(obj->mFileBuffer + framesRead,
                                    framesToRead - framesRead);
      }
    }
    //    size_t written =
    obj->mRingBuffer->write((const char *)obj->mFileBuffer,
                            framesRead * sizeof(float) * obj->channels());
    //    if (written != framesRead * sizeof(float) * obj->channels()) {
    //      std::cerr << "Error writing to ring buffer" << std::endl;
    //    }
    if (obj->mReadCallback) {
      obj->mReadCallback(obj->mFileBuffer, obj->mSf.channels(), framesRead,
                         obj->mCallbackData);
    }
    lk.unlock();
  }
}

gam::SoundFile::EncodingType SoundFileBuffered::encoding() const {
  if (opened()) {
    return mSf.encoding();
  } else {
    return (gam::SoundFile::EncodingType)0;
  }
}

gam::SoundFile::Format SoundFileBuffered::format() const {
  if (opened()) {
    return mSf.format();
  } else {
    return (gam::SoundFile::Format)0;
  }
}

double SoundFileBuffered::frameRate() const {
  if (opened()) {
    return mSf.frameRate();
  } else {
    return 0.0;
  }
}

int SoundFileBuffered::frames() const {
  if (opened()) {
    return mSf.frames();
  } else {
    return 0;
  }
}

int SoundFileBuffered::channels() const {
  if (opened()) {
    return mSf.channels();
  } else {
    return 0;
  }
}

int SoundFileBuffered::samples() const {
  if (opened()) {
    return mSf.samples();
  } else {
    return 0;
  }
}

int SoundFileBuffered::repeats() { return mRepeats.load(); }

void SoundFileBuffered::setReadCallback(SoundFileBuffered::CallbackFunc func,
                                        void *userData) {
  mReadCallback = func;
  mCallbackData = userData;
}

void SoundFileBuffered::seek(int frame) {
  if (frame < 0) {
    frame = 0;
  }
  if (frame >= frames()) {
    frame = frames() - 1;
  }
  mSeek.store(frame);
  mCurPos.store(frame);
}

int SoundFileBuffered::currentPosition() { return mCurPos.load(); }
